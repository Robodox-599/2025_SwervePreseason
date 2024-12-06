package frc.robot.subsystems.vision;


import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public interface VisionIO{
    @AutoLog
    public static class VisionIOInputs {
    
        /**
         * Vision system constants, including the camera name, transform from the robot to the camera,
         * and a standard deviation factor for measurements.
         */
        public VisionConstants constants = 
            new VisionConstants(
                "Default",
                new Transform3d(new Translation3d(), new Rotation3d()),
                1.0
            );
    
        /**
         * Indicates whether the camera is currently connected.
         */
        public boolean cameraConnected = false;
    
        /**
         * Stores the latest observed angles of a target as a pair of 2D rotations.
         */
        public ObservedTargetRotations latestTargetAngle = 
            new ObservedTargetRotations(new Rotation2d(), new Rotation2d());
    
        /**
         * An array containing pose observations made by the camera. Each observation includes
         * metadata such as timestamp, observed pose, ambiguity, and tag information.
         */
        public PoseObservation[] poseObservations = new PoseObservation[0];
    
        /**
         * The number of targets currently detected by the vision system.
         */
        public double numTargets = 0;
    
        /**
         * An array of IDs corresponding to the AprilTags detected by the vision system.
         */
        public int[] aprilTagIds = new int[0];
    }

    /**
     * Represents an observed pose of a target as a pair of 2D rotations (targetX, targetY).
     * This is used for tracking the orientation or position of a detected target.
     
     * @param targetX the angle around the X-axis
     * @param targetY the angle around the Y-axis
     */
    public static record ObservedTargetRotations(Rotation2d targetX, Rotation2d targetY){}

    /**
     * Represents a single pose observation from the camera. 
     * Includes information about the observed pose and additional metadata.

     * @param timestamp the time the pose was observed
     * @param observedPose the 3D pose detected by the camera
     * @param ambiguity a measure of how uncertain the observation is
     * @param tagCount the number of tags detected contributing to this pose
     * @param averageTagDistance the average distance to the detected tags
     */
    public static record PoseObservation(double timestamp, Pose3d observedPose, double ambiguity, int tagCount, double averageTagDistance) {}

    /** 
     * @return Name of camera instance applied to the camera (grabbed from VisionConsants) 
     */
    public String getName();

    
    /** 
     * Updates Inputs to the given values from the IO layers 
     */
    public default void updateInputs(VisionIOInputs inputs) {}

    /** 
     * @return Return VisionConstants of Camera.  
     */
    public VisionConstants getVisionConstants();
}