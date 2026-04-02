from typing import Callable
from math import hypot

from commands2 import Subsystem
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d
from wpimath.units import inchesToMeters

from phoenix6 import swerve

from photonlibpy import EstimatedRobotPose, PhotonCamera, PhotonPoseEstimator

class Vision(Subsystem):
    """
    Fuse AprilTag pose estimates from the robot's PhotonVision cameras.
    """

    def __init__(self, add_vision_measurement, 
                 get_current_swerve_state: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState],
                 get_robot_tilt: Callable[[], tuple[float, float]],
                 field_type: str,
                 linear_std_dev_baseline: float, angular_std_dev_baseline: float,
                 camera_std_dev_factors: tuple[float, ...],max_linear_speed: float,
                 max_angular_speed: float, max_tilt_deg: float):
        """
        Construct the vision subsystem and per-camera pose estimators.

        :param add_vision_measurement: Callback used to inject accepted vision measurements into drivetrain odometry.
        :type add_vision_measurement: Callable[[wpimath.geometry.Pose2d, float, tuple[float, float, float]], None]
        :param get_current_swerve_state: Function that returns the current drivetrain state.
        :type get_current_swerve_state: Callable[[], phoenix6.swerve.SwerveDrivetrain.SwerveDriveState]
        :param get_robot_tilt: Function that returns the current robot pitch and roll in degrees.
        :type get_robot_tilt: Callable[[], tuple[float, float]]
        :param field_type: Name of the active field variant used to load the AprilTag layout.
        :type field_type: str
        :param linear_std_dev_baseline: Baseline linear measurement standard deviation in meters.
        :type linear_std_dev_baseline: float
        :param angular_std_dev_baseline: Baseline angular measurement standard deviation in radians.
        :type angular_std_dev_baseline: float
        :param camera_std_dev_factors: Per-camera multipliers applied to the baseline standard deviations.
        :type camera_std_dev_factors: tuple[float, ...]
        :param max_linear_speed: Linear speed threshold above which vision trust is reduced.
        :type max_linear_speed: float
        :param max_angular_speed: Angular speed threshold above which vision trust is reduced.
        :type max_angular_speed: float
        :param max_tilt_deg: Tilt threshold in degrees beyond which measurements are rejected.
        :type max_tilt_deg: float
        """
        
        Subsystem.__init__(self)

        self.add_vision_measurement = add_vision_measurement
        self.get_current_swerve_state = get_current_swerve_state
        self.get_robot_tilt = get_robot_tilt
        self.field_type = field_type
        self.april_tag_layout = self._load_april_tag_layout(field_type)
        self.linear_std_dev_baseline = linear_std_dev_baseline
        self.angular_std_dev_baseline = angular_std_dev_baseline
        self.camera_std_dev_factors = camera_std_dev_factors
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_tilt_deg = max_tilt_deg

        self.back_camera = PhotonCamera("back_camera")
        self.front_left_camera = PhotonCamera("front_left_camera")
        self.front_right_camera = PhotonCamera("front_right_camera")

        robot_to_back_camera_translation = Transform3d(
            Translation3d(inchesToMeters(4.25), inchesToMeters(10), inchesToMeters(20.5)),
            Rotation3d.fromDegrees(1, 0, 180),
        )
        robot_to_front_left_camera_translation = Transform3d(
            Translation3d(inchesToMeters(13.5), inchesToMeters(12.5), inchesToMeters(20.25)),
            Rotation3d.fromDegrees(-1, 0, 0),
        )
        robot_to_front_right_camera_translation = Transform3d(
            Translation3d(inchesToMeters(13.5), inchesToMeters(-12.25), inchesToMeters(20.25)),
            Rotation3d.fromDegrees(-4, 0, 0),
        )

        self.back_camera_pose_est = PhotonPoseEstimator(
            self.april_tag_layout, robot_to_back_camera_translation
        )
        self.front_left_camera_pose_est = PhotonPoseEstimator(
            self.april_tag_layout, robot_to_front_left_camera_translation
        )
        self.front_right_camera_pose_est = PhotonPoseEstimator(
            self.april_tag_layout, robot_to_front_right_camera_translation
        )

        self.cameras = [
            (self.back_camera, self.back_camera_pose_est),
            (self.front_left_camera, self.front_left_camera_pose_est),
            (self.front_right_camera, self.front_right_camera_pose_est),
        ]

    def _load_april_tag_layout(self, field_type: str) -> AprilTagFieldLayout:
        """
        Load the AprilTag layout matching the configured field variant.

        :param field_type: Name of the active field variant.
        :type field_type: str
        :returns: AprilTag field layout for the requested field variant.
        :rtype: robotpy_apriltag.AprilTagFieldLayout
        """
        field_layouts = {
            "AndyMark": AprilTagField.k2026RebuiltAndyMark,
            "Welded": AprilTagField.k2026RebuiltWelded,
        }
        april_tag_field = field_layouts.get(field_type)

        return AprilTagFieldLayout.loadField(april_tag_field)

    def periodic(self):
        """
        Poll each camera and push accepted pose measurements into odometry.
        """
        current_state = self.get_current_swerve_state()

        for camera_index, (camera, pose_est) in enumerate(self.cameras):
            measurement = self.get_vision_measurement(
                camera_index,
                camera,
                pose_est,
                current_state
            )

            if measurement == None:
                continue

            pose, timestamp, std_devs = measurement
            self.add_vision_measurement(pose, timestamp, std_devs)

    def reject_pose_estimate(self, pose: Pose3d, current_state: swerve.SwerveDrivetrain.SwerveDriveState) -> bool:
        """
        Reject pose estimates that are outside the field or current motion limits.

        :param pose: Estimated robot pose from PhotonVision.
        :type pose: wpimath.geometry.Pose3d
        :param current_state: Current swerve drivetrain state used for motion gating.
        :type current_state: phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
        :returns: True when the pose estimate should be rejected.
        :rtype: bool
        """
        current_vx = abs(current_state.speeds.vx)
        current_vy = abs(current_state.speeds.vy)
        current_omega = abs(current_state.speeds.omega)
        pitch_deg, roll_deg = self.get_robot_tilt()

        return not (
            -0.25 < pose.Z() < 0.5
            and 0.0 < pose.X() < self.april_tag_layout.getFieldLength()
            and 0.0 < pose.Y() < self.april_tag_layout.getFieldWidth()
            and current_vx <= self.max_linear_speed
            and current_vy <= self.max_linear_speed
            and current_omega <= self.max_angular_speed
            and abs(pitch_deg) <= self.max_tilt_deg
            and abs(roll_deg) <= self.max_tilt_deg
        )

    def get_average_tag_distance(self, estimated_pose: EstimatedRobotPose) -> float | None:
        """
        Compute the mean robot-to-tag distance for the tags used in a solve.

        :param estimated_pose: Multi-tag pose estimate returned by PhotonVision.
        :type estimated_pose: photonlibpy.EstimatedRobotPose
        :returns: Average robot-to-tag distance in meters, or None when no valid tags are present.
        :rtype: float | None
        """
        robot_translation = estimated_pose.estimatedPose.toPose2d().translation()

        total_distance = 0.0
        tag_count = 0

        for target in estimated_pose.targetsUsed:
            tag_pose = self.april_tag_layout.getTagPose(target.getFiducialId())
            if tag_pose is None:
                continue

            total_distance += robot_translation.distance(tag_pose.toPose2d().translation())
            tag_count += 1

        if tag_count == 0:
            return None

        return total_distance / tag_count 

    def speed_factor(self, current_speed: float, max_speed: float) -> float:
        """
        Scale vision trust upward once the robot exceeds half of the speed limit.

        :param current_speed: Current robot speed for the axis being evaluated.
        :type current_speed: float
        :param max_speed: Maximum speed used as the scaling reference.
        :type max_speed: float
        :returns: Scalar applied to the baseline standard deviation.
        :rtype: float
        """
        speed_ratio = current_speed / max_speed
        if speed_ratio <= 0.5:
            return 1.0
        else:
            return speed_ratio / 0.5
    
    def calc_std_dev(self, estimated_pose: EstimatedRobotPose, current_state: swerve.SwerveDrivetrain.SwerveDriveState,
                     camera_index: int):
        """
        Estimate measurement standard deviations for a camera pose solution.

        :param estimated_pose: Pose estimate returned by PhotonVision.
        :type estimated_pose: photonlibpy.EstimatedRobotPose
        :param current_state: Current drivetrain state used to scale measurement trust.
        :type current_state: phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
        :param camera_index: Index of the camera that produced the estimate.
        :type camera_index: int
        :returns: Standard deviation tuple for x, y, and heading, or None when the estimate is unusable.
        :rtype: tuple[float, float, float] | None
        """
        avg_tag_distance = self.get_average_tag_distance(estimated_pose)
        current_linear_speed = abs(hypot(current_state.speeds.vx, current_state.speeds.vy))
        current_angular_speed = abs(current_state.speeds.omega)

        tag_count = len(estimated_pose.targetsUsed)
        if avg_tag_distance is None or tag_count == 0:
            return None

        distance_factor = (avg_tag_distance ** 2.0) / tag_count

        linear_speed_factor = self.speed_factor(current_linear_speed, self.max_linear_speed)
        angular_speed_factor = self.speed_factor(current_angular_speed, self.max_angular_speed)

        camera_factor = (
            self.camera_std_dev_factors[camera_index]
            if camera_index < len(self.camera_std_dev_factors)
            else 1.0
        )

        linear_std_dev = (
            self.linear_std_dev_baseline
            * distance_factor
            * camera_factor
            * linear_speed_factor
        )

        angular_std_dev = (
            self.angular_std_dev_baseline
            # * distance_factor
            # * camera_factor
            # * angular_speed_factor
        )

        return (linear_std_dev, linear_std_dev, angular_std_dev)

    def get_vision_measurement(self, camera_index: int, camera: PhotonCamera, pose_est: PhotonPoseEstimator,
                               current_state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Return the latest accepted pose measurement from one camera, if any.

        :param camera_index: Index of the camera in the subsystem camera list.
        :type camera_index: int
        :param camera: PhotonCamera instance to read from.
        :type camera: photonlibpy.PhotonCamera
        :param pose_est: PhotonPoseEstimator associated with the camera.
        :type pose_est: photonlibpy.PhotonPoseEstimator
        :param current_state: Current drivetrain state used for gating and covariance scaling.
        :type current_state: phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
        :returns: Accepted pose measurement tuple, or None when no usable result exists.
        :rtype: tuple[wpimath.geometry.Pose2d, float, tuple[float, float, float] | None] | None
        """
        results = camera.getAllUnreadResults()
        if len(results) == 0:
            return None

        latest_result = results[-1]

        if len(latest_result.getTargets()) <= 1:
            return None

        pose = pose_est.estimateCoprocMultiTagPose(latest_result)

        if pose is None or self.reject_pose_estimate(pose.estimatedPose, current_state):
            return None

        return (
            pose.estimatedPose.toPose2d(),
            pose.timestampSeconds,
            self.calc_std_dev(pose, current_state, camera_index)
        )
