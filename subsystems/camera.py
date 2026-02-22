from photonlibpy import PhotonCamera, PhotonPoseEstimator

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from wpimath.geometry import Transform3d, Translation3d, Rotation3d

class VisionCamera():
    def __init__(self):
        self.back_camera = PhotonCamera("front_camera")

        robot_to_camera_translation = Transform3d(
            Translation3d(0.3175, 0.26035, 0.511175),
            Rotation3d.fromDegrees(0, -88.5, 0),
        )

        # For districts in Wisconsin, AndyMark fields are used.
        # For regionals in the U.S., Welded fields are used.
        # REMEMBER TO CHANGE THIS IN CODE AND GUI DEPENDING ON COMPETITION TYPE!
        self.pose_est = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark),
            robot_to_camera_translation,
        )

    def get_vision_measurement(self):
        results = self.back_camera.getAllUnreadResults()

        pose = None
        for result in results:
            num_targets = len(result.getTargets())
            if num_targets > 1:
                pose = self.pose_est.estimateCoprocMultiTagPose(result)
            elif num_targets == 1:
                pose = self.pose_est.estimateLowestAmbiguityPose(result)                

        if pose != None:
            return pose.estimatedPose.toPose2d(), pose.timestampSeconds
        else: 
            return None, None
