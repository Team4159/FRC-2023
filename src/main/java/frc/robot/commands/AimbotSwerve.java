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
import frc.robot.FieldRegion;
import frc.robot.subsystems.Swerve;

public class AimbotSwerve extends SequentialCommandGroup {
    public AimbotSwerve(Swerve s_Swerve, FieldRegion region) {
        Pose2d pose = region.getTargetPose();
        if (pose == null) {
            DriverStation.reportWarning("TARGET NOT FOUND", false);
            return;
        }

        addCommands(
            TrajectoryCommand.generate(s_Swerve, generateStraightTrajectory(s_Swerve.getPose(), pose)),
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
