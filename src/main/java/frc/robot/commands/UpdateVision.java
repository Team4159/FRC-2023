package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class UpdateVision extends SubsystemBase {
    private Swerve swerve;

    public UpdateVision(Swerve swerve) {
        this.swerve = swerve;

        //addRequirements(swerve); // ensure the swerve subsystem has loaded first
    }

    public void periodic() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight"); // Fetches the limelight section of the networktables
        double[] pose = limelight.getEntry("botpose").getDoubleArray(new double[6]);// gets the pose portion as an array of doubles [x, y, z, rotX, rotY, rotZ]
        if (pose.length == 0)
            return;
        
        System.out.println(limelight.getEntry("tl").getDouble(0));

        swerve.updatePoseEstimator( // sends new data to swerve
            new Pose2d(pose[0], pose[1], new Rotation2d( pose[5])), // TODO: check if radians/degrees and if this is the right axis of rotation
            limelight.getEntry("tl").getDouble(0)
        );
    }
}
