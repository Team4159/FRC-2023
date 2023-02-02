package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.AutoMode;

/*
 * Hello, Shuffleboard team!
 * This is the class you'll be working with, and you will be adding different functions to fetch information from relevant subsystems
 * and post them to the Shuffleboard with the SmartDashboard class!
 *  - Max
 */
public class DataBoard extends SubsystemBase {
    private Field2d field = new Field2d(); // initialize the field for pose estimation
    private Field2d visionField = new Field2d(); // initialize the field for pose estimation

    public DataBoard() {
        SmartDashboard.putData("field", field); // send the field object to the shuffleboard
        SmartDashboard.putData("visionField", visionField);
        DriverStation.getMatchTime();
    }

    public AutoMode getAutoMode() { // fetches whether to dock or not during auto, according to a button
        return SmartDashboard.setDefaultBoolean("autoDock", false) ? AutoMode.Dock : AutoMode.Normal;
    }

    public void setVisionPose(Pose2d pose) {
        visionField.setRobotPose(pose);
    }

    @Override
    public void periodic() {
        field.setRobotPose(RobotContainer.s_Swerve.getPose()); // updates the robot's estimated position on the field
        SwerveModulePosition[] positions = RobotContainer.s_Swerve.getPositions();
        for(int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("swerveMod" + i, positions[i].angle.getDegrees());
        }

        SmartDashboard.putNumber("Game Timer", DriverStation.getMatchTime());
        // SmartDashboard.putNumber("swerveMod0", positions[0].angle.getDegrees());

    }
}
