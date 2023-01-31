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

        addRequirements(swerve); // ensure the swerve subsystem has loaded first
    }

    @Override
    public void execute() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight"); // Fetches the limelight section of the networktables
        double[] pose = limelight.getEntry("botpose").getValue().getDoubleArray();// gets the pose portion as an array of doubles [x, y, z, rotX, rotY, rotZ]
        if (pose.length == 0) return;
        swerve.updatePoseEstimator( // sends new data to swerve
            new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5])),
            limelight.getEntry("tl").getDouble(0)
        );
    }
}
