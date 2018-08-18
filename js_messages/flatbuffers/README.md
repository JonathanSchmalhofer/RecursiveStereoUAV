Call for image and dependencies:
    flatc --cpp Image.fbs Header.fbs time.fbs

Call for trajectory and dependencies:
    flatc --cpp Trajectory3D.fbs Trajectory3DPointStamped.fbs time_ros_to_airsim.fbs Pose.fbs Point.fbs Quaternion.fbs

Call for trajectory3d point as root and dependencies:
    flatc --cpp Trajectory3DPointStampedRoot.fbs time_ros_to_airsim.fbs Pose.fbs Point.fbs Quaternion.fbs
    
Call for Pose from airsim to ros as root and dependencies:
    flatc --cpp PoseMessageRoot.fbs Header.fbs time.fbs

Call for StereoImagePose and dependencies:
    flatc --cpp StereoImagePose.fbs Header.fbs time.fbs StereoImage.fbs PoseRPY.fbs PointRPY.fbs OrientationRPY.fbs