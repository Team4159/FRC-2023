package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TrajectoryAuto extends SwerveControllerCommand { // trust the process
    private static ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints
    );
    static {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public TrajectoryAuto(Swerve s, Trajectory t) {
        super(t, s::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s::setModuleStates, s
        );
    }
}
