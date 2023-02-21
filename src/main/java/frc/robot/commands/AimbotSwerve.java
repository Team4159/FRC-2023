package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldRegion;
import frc.robot.subsystems.Swerve;

public class AimbotSwerve extends SequentialCommandGroup {

    private FieldRegion region;
    private Pose2d pose;

    private Swerve s_Swerve;

    public AimbotSwerve(Swerve s_Swerve) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);


        addCommands(
            new InstantCommand(() -> setRegion()),
            new PrintCommand("pose: " + pose),
            new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
            //s_Swerve.followTrajectoryCommand(generateStraightTrajectory(pose, pose), false),
            new InstantCommand(() -> s_Swerve.resetModulesToAbsolute())
        );
    }

    private void setRegion() {
        region = FieldRegion.lookup(s_Swerve.getPose());
        pose = region.getTargetPose();
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