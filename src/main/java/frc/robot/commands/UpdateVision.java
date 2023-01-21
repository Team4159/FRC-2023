package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class UpdateVision extends CommandBase {
    private Swerve swerve;

    public UpdateVision(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    public void execute() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        double[] pose = limelight.getValue("pose").getDoubleArray();
        swerve.updatePoseEstimator(
            new Pose2d(pose[0], pose[1], new Rotation2d(pose[5])),
            limelight.getEntry("tl").getDouble(0)
        );
    }
}