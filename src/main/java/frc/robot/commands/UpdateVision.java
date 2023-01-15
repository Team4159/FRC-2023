package frc.robot.commands;

// import frc.robot.Constants;
// import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdateVision extends CommandBase {
    private Swerve swerve;

    public UpdateVision(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    public void execute() {
        // NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        // swerve.updatePoseEstimator(new Pose2d(
            
        // ), limelight.getEntry("tl").getDouble(0));
        // .getEntry("<variablename>").getDouble(0); TODO: Get pose estimation
    }
}
