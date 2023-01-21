package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

/*
 * Hello, Auto Team!
 * This is an example autonomous routine which you will be copying to add features to!
 *  - Max
 */
public class ExampleAuto extends SequentialCommandGroup {
    public ExampleAuto(Swerve s_Swerve) {
        String trajectoryJSON = "Red1Dock";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            
            addCommands(
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
                new TrajectoryAuto(s_Swerve, trajectory)
            );
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }
}