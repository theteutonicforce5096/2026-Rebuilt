from photonlibpy import PhotonCamera, PhotonPoseEstimator

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from wpimath.geometry import Transform3d, Translation3d, Rotation3d

class VisionCamera():
    def __init__(self):
        self.back_camera = PhotonCamera("back_camera")

        robot_to_camera_translation = Transform3d(
            Translation3d(-0.3175, 0.009525, 0.46355),
            Rotation3d.fromDegrees(0, 0, 180),
        )

        # For districts in Wisconsin, AndyMark fields are used.
        # For regionals in the U.S., Welded fields are used.
        # REMEMBER TO CHANGE THIS DEPENDING ON COMPETITION TYPE!
        self.pose_est = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark),
            robot_to_camera_translation,
        )

    def get_vision_measurement(self):
        pose = self.pose_est.estimateCoprocMultiTagPose(self.back_camera.getLatestResult())
        if pose:
            return pose.estimatedPose, pose.timestampSeconds
        else: 
            return None, None
