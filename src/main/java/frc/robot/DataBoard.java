package frc.robot;

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

    public DataBoard() {
        SmartDashboard.putData("field", field); // send the field object to the shuffleboard
    }

    public AutoMode getAutoMode() { // fetches whether to dock or not during auto, according to a button
        return SmartDashboard.setDefaultBoolean("autoDock", false) ? AutoMode.Dock : AutoMode.Normal;
    }

    @Override
    public void periodic() {
        field.setRobotPose(RobotContainer.s_Swerve.getPose()); // updates the robot's estimated position on the field
    }
}
