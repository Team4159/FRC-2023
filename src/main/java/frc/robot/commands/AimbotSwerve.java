package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AimbotSwerve extends SequentialCommandGroup {

    private Trajectory trajectory;

    public AimbotSwerve(Swerve s_Swerve, Pose2d targetPose) {

        if (targetPose == null) {
            System.err.println("TARGET NOT FOUND");
            return;
        }

        trajectory = generateStraightTrajectory(s_Swerve.getPose(), targetPose);


        var thetaController = new ProfiledPIDController( // PID controller for rotation
            Constants.AutoConstants.kPThetaController, 0, 0,
            Constants.AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI); // shortest rotation path so the robot doesn't try to turn more than 180 degrees


        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand( // Command to tell swerve to follow trajectory
            trajectory,
            s_Swerve::getPose, // supplies the robot pose
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),  // PID controller for x position
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),  // PID controller for y position
            thetaController, // PID controller for rotation
            s_Swerve::setModuleStates, // tell the modules to do things
            s_Swerve // require swerve subsystem
        );


        addCommands(swerveControllerCommand);

    }

    public Trajectory generateStraightTrajectory(Pose2d initialPose, Pose2d targetPose) {

        TrajectoryConfig config = new TrajectoryConfig( // config for trajectory -- max speed and acceleration
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics
        );

        return TrajectoryGenerator.generateTrajectory( // generates the straight trajectory between initial and target pose
            initialPose, // start at initial pose
            null, // straight path so no intermediate waypoints
            targetPose, // end at target pose
            config
        );

    }
}
