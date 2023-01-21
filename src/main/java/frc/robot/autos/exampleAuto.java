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
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON); // find out where your trajectory is in the files
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath); // load the json into a trajectory
            
            addCommands( // create the sequence of subcommands that will be running
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())), // use an inline function to reset the odometry to your initial position
                new TrajectoryAuto(s_Swerve, trajectory) // the actual command that asks the swerve to run along this trajectory
            );
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }
}