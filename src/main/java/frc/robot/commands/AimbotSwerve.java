package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
            new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
            s_Swerve.followTrajectoryCommand(generateStraightTrajectory(s_Swerve.getPose(), pose), false),
            new InstantCommand(() -> s_Swerve.resetModulesToAbsolute())
        );
    }

    public static PathPlannerTrajectory generateStraightTrajectory(Pose2d initialPose, Pose2d targetPose) {
        return PathPlanner.generatePath( // TODO: account for initial velocity
            Constants.AutoConstants.kPathConstraints, 
            new PathPoint(initialPose.getTranslation(), getHeading(initialPose, targetPose), initialPose.getRotation()),
            new PathPoint(targetPose.getTranslation(), getHeading(targetPose, initialPose), targetPose.getRotation())
        );
    }

    public static Rotation2d getHeading(Pose2d one, Pose2d two) {
        return Rotation2d.fromRadians(
            Math.atan2(two.getY() - one.getY(), two.getX() - one.getX())
        );
    }
}
