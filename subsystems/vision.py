from photonlibpy import PhotonCamera, PhotonPoseEstimator, EstimatedRobotPose

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d

from commands2 import Subsystem

class VisionCamera(Subsystem):
    def __init__(self, add_vision_measurement: callable):
        Subsystem.__init__(self)

        self.add_vision_measurement = add_vision_measurement

        # For districts in Wisconsin, AndyMark fields are used.
        # For regionals in the U.S., Welded fields are used.
        # REMEMBER TO CHANGE THIS IN CODE AND GUI DEPENDING ON COMPETITION TYPE!
        # https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-andymark.json
        self.april_tag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)

        self.back_camera = PhotonCamera("front_camera")
        self.front_left_camera = PhotonCamera("front_camera")
        self.front_right_camera = PhotonCamera("front_camera")

        robot_to_back_camera_translation = Transform3d(
            Translation3d(0.3175, 0.26035, 0.511175),
            Rotation3d.fromDegrees(0, -88.5, 0),
        )
        robot_to_front_left_camera_translation = Transform3d(
            Translation3d(0.3175, 0.26035, 0.511175),
            Rotation3d.fromDegrees(0, -88.5, 0),
        )
        robot_to_front_right_camera_translation = Transform3d(
            Translation3d(0.3175, 0.26035, 0.511175),
            Rotation3d.fromDegrees(0, -88.5, 0),
        )

        self.back_camera_pose_est = PhotonPoseEstimator(
            self.april_tag_layout,
            robot_to_back_camera_translation
        )
        self.front_left_camera_pose_est = PhotonPoseEstimator(
            self.april_tag_layout,
            robot_to_front_left_camera_translation
        )
        self.front_right_camera_pose_est = PhotonPoseEstimator(
            self.april_tag_layout,
            robot_to_front_right_camera_translation
        )

        self.cameras = [
            (self.back_camera, self.back_camera_pose_est), 
            (self.front_left_camera, self.front_left_camera_pose_est), 
            (self.front_right_camera, self.front_right_camera_pose_est)
        ]
        


    def periodic(self):
        for camera, pose_est in self.cameras:
            pose, timestamp = self.get_vision_measurement(camera, pose_est)
            if pose != None:
                self.add_vision_measurement(pose, timestamp)

    def reject_pose_estimate(self, pose: Pose3d):
        reject_pose = (
            # Must be within a reasonable height range (not above the trench or under the floor)
            -0.25 < pose.Z() < 0.5
            
            # Must be within the field boundaries
            or 0.0 < pose.X() < self.april_tag_layout.getFieldLength() 
            or 0.0 < pose.Y() < self.april_tag_layout.getFieldWidth()
        )

        return reject_pose

    def calc_std_dev(self, pose: EstimatedRobotPose):
        std_dev_factor = (pose.estimatedPose.() ** 2.0) / observation.tag_count()
        linear_std_dev = linear_std_dev_baseline * std_dev_factor
        angular_std_dev = angular_std_dev_baseline * std_dev_factor

        if camera_index < len(camera_std_dev_factors):
            linear_std_dev *= camera_std_dev_factors[camera_index]
            angular_std_dev *= camera_std_dev_factors[camera_index]

    def get_vision_measurement(self, camera: PhotonCamera, pose_est: PhotonPoseEstimator):
        results = camera.getAllUnreadResults()
        if len(results) == 0:
            return None, None, None
            
        latest_result = results[-1]

        pose = None
        latest_result.
        num_targets = len(latest_result.getTargets())
        if num_targets > 1:
            pose = pose_est.estimateCoprocMultiTagPose(latest_result)    

        if pose != None and not self.reject_pose_estimate(pose.estimatedPose):
            return pose.estimatedPose.toPose2d(), pose.timestampSeconds, self.calc_std_dev(pose) 
        
        return None, None, None