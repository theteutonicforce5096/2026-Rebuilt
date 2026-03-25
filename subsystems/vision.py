from typing import Callable
from math import hypot

from commands2 import Subsystem
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d
from wpimath.units import inchesToMeters

from phoenix6 import swerve

from photonlibpy import EstimatedRobotPose, PhotonCamera, PhotonPoseEstimator

class Vision(Subsystem):
    def __init__(self, add_vision_measurement, 
                 get_current_swerve_state: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState],
                 get_robot_tilt: Callable[[], tuple[float, float]],
                 field_type: str,
                 linear_std_dev_baseline: float, angular_std_dev_baseline: float,
                 camera_std_dev_factors: tuple[float, ...],max_linear_speed: float,
                 max_angular_speed: float, max_tilt_deg: float):
        
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

    def _load_april_tag_layout(field_type: str) -> AprilTagFieldLayout:
        field_layouts = {
            "AndyMark": AprilTagField.k2026RebuiltAndyMark,
            "Welded": AprilTagField.k2026RebuiltWelded,
        }
        april_tag_field = field_layouts.get(field_type)

        return AprilTagFieldLayout.loadField(april_tag_field)

    def periodic(self):
        current_state = self.get_current_swerve_state()

        for camera_index, (camera, pose_est) in enumerate(self.cameras):
            measurement = self.get_vision_measurement(
                camera_index,
                camera,
                pose_est,
                current_state,
            )

            if measurement == None:
                continue

            pose, timestamp, std_devs = measurement
            self.add_vision_measurement(pose, timestamp, std_devs)

    def reject_pose_estimate(self, current_state: swerve.SwerveDrivetrain.SwerveDriveState, pose: Pose3d) -> bool:
        current_linear_speed = hypot(current_state.speeds.vx, current_state.speeds.vy)
        current_angular_speed = current_state.speeds.omega
        pitch_deg, roll_deg = self.get_robot_tilt()

        return not (
            -0.25 < pose.Z() < 0.5
            and 0.0 < pose.X() < self.april_tag_layout.getFieldLength()
            and 0.0 < pose.Y() < self.april_tag_layout.getFieldWidth()
            and current_linear_speed <= self.max_linear_speed
            and abs(current_angular_speed) <= self.max_angular_speed
            and abs(pitch_deg) <= self.max_tilt_deg
            and abs(roll_deg) <= self.max_tilt_deg
        )

    def get_average_tag_distance(self, estimated_pose: EstimatedRobotPose) -> float | None:
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

    def calc_std_dev(self, estimated_pose: EstimatedRobotPose, camera_index: int):
        avg_tag_distance = self.get_average_tag_distance(estimated_pose)

        tag_count = len(estimated_pose.targetsUsed)
        if avg_tag_distance is None or tag_count == 0:
            return None

        distance_factor = (avg_tag_distance ** 2.0) / tag_count

        camera_factor = (
            self.camera_std_dev_factors[camera_index]
            if camera_index < len(self.camera_std_dev_factors)
            else 1.0
        )

        linear_std_dev = (
            self.linear_std_dev_baseline
            * distance_factor
            * camera_factor
        )

        angular_std_dev = (
            self.angular_std_dev_baseline
            * distance_factor
            * camera_factor
        )

        return (linear_std_dev, linear_std_dev, angular_std_dev)

    def get_vision_measurement(self, camera_index: int, camera: PhotonCamera, pose_est: PhotonPoseEstimator,
                               current_state: swerve.SwerveDrivetrain.SwerveDriveState):
        results = camera.getAllUnreadResults()
        if len(results) == 0:
            return None

        latest_result = results[-1]

        if len(latest_result.getTargets()) <= 1:
            return None

        pose = pose_est.estimateCoprocMultiTagPose(latest_result)

        if pose is None or self.reject_pose_estimate(current_state,pose.estimatedPose):
            return None

        return (
            pose.estimatedPose.toPose2d(),
            pose.timestampSeconds,
            self.calc_std_dev(pose, camera_index),
        )
