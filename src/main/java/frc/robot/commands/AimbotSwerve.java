package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AimbotSwerve extends SequentialCommandGroup {

    private Trajectory trajectory;

    public AimbotSwerve(Swerve s_Swerve, Pose2d targetPose) {
        if (targetPose == null) {
            DriverStation.reportWarning("TARGET NOT FOUND", false);
            return;
        }

        trajectory = generateStraightTrajectory(s_Swerve.getPose(), targetPose);

        addCommands(
            TrajectoryCommand.generate(s_Swerve, trajectory),
            new InstantCommand(() -> s_Swerve.resetModulesToAbsolute())
        );
    }

    public static Trajectory generateStraightTrajectory(Pose2d initialPose, Pose2d targetPose) {
        TrajectoryConfig config = new TrajectoryConfig( // config for trajectory -- max speed and acceleration
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(Constants.Swerve.swerveKinematics);

        return TrajectoryGenerator.generateTrajectory( // generates the straight trajectory between initial and target pose
            initialPose, // start at initial pose
            List.of(),  // straight path so no intermediate waypoints
            targetPose, // end at target pose
            config
        );
    }

}
