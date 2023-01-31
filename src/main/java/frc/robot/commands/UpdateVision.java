package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DataBoard;
import frc.robot.subsystems.Swerve;

public class UpdateVision extends CommandBase {
    private Swerve swerve;
    private DataBoard board;

    public UpdateVision(Swerve swerve, DataBoard board) {
        this.swerve = swerve;
        this.board = board;

        addRequirements(swerve, board); // ensure the swerve subsystem has loaded first
    }

    @Override
    public void execute() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight"); // Fetches the limelight section of the networktables
        double[] poseData = limelight.getEntry("botpose").getValue().getDoubleArray(); // gets the pose portion as an array of doubles [x, y, z, rotX, rotY, rotZ]
        if (poseData.length == 0) return;
        var pose = new Pose2d(poseData[0], poseData[1], Rotation2d.fromDegrees(poseData[5]));
        swerve.updatePoseEstimator(pose, limelight.getEntry("tl").getDouble(0));// sends new data to swerve
        board.setVisionPose(pose);
    }
}
