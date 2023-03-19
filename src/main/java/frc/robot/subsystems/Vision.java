package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight"); // Fetches the limelight section of the networktables

    public void setPipeline(int num) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(num);
    }

    public long getPipeline() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getInteger(-1);
    }

    @Override
    public void periodic() {
        double tX = limelight.getEntry("tx").getDouble(0); // gets the horizontal offset in degrees between the crosshair and the target
        RobotContainer.s_Swerve.setConeTx(tX);



        // double[] poseData = limelight.getEntry("botpose").getValue().getDoubleArray(); // gets the pose portion as an array of doubles [x, y, z, rotX, rotY, rotZ, total latency]
        // if (poseData.length != 7) return;
        // var pose = new Pose2d(poseData[0] + Constants.VisionConstants.fieldWidth/2, poseData[1] + Constants.VisionConstants.fieldHeight/2, Rotation2d.fromDegrees(poseData[5]));
        // RobotContainer.s_Swerve.updatePoseEstimator(pose,poseData[6]); // sends new data to swerve
        // RobotContainer.dataBoard.setVisionPose(pose);
    }

    // public static class SwapVisionPipeline extends CommandBase {
    //     private int num;
        
    //     public SwapVisionPipeline(int num) {
    //         this.num = num;
    //     }
    
    //     @Override
    //     public void execute() { // send the new pipeline # to NetworkTables
    //         NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(num);
    //     }
    
    //     @Override
    //     public boolean isFinished() { // check whether the pipeline has in fact been changed
    //         return NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getInteger(-1) == num;
    //     }
    // }
}