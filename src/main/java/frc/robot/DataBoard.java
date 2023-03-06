package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer.AutoMode;


/*
 * Hello, Shuffleboard team!
 * This is the class you'll be working with, and you will be adding different functions to fetch information from relevant subsystems
 * and post them to the Shuffleboard with the SmartDashboard class!
 *  - Max
 */
public class DataBoard {
    private final SendableChooser<Integer> autoSelector = new SendableChooser<>();
    private Field2d field = new Field2d(); // initialize the field for pose estimation
    private Field2d visionField = new Field2d(); // initialize the field for pose estimation

    public void init() {
        autoSelector.setDefaultOption("Disabled", -1);
        autoSelector.addOption("Autobalance",0);
        autoSelector.addOption("Position 1",1);
        autoSelector.addOption("Position 2", 2);
        autoSelector.addOption("Position 3", 3);

        SmartDashboard.putData("Auto Selector", autoSelector);
        SmartDashboard.putData("field", field); // send the field object to the shuffleboard
        SmartDashboard.putData("visionField", visionField);
        Shuffleboard.getTab("Testing").addDouble("Pigeon2Gyro", () -> RobotContainer.s_Swerve.gyro.getYaw()).withWidget(BuiltInWidgets.kGyro).close();
    }


    public AutoMode getAutoMode() { // fetches whether to dock or not during auto, according to a button
        return SmartDashboard.setDefaultBoolean("autoDock", false) ? AutoMode.Dock : AutoMode.Normal;
    }


    public int getAutoPos() {
        return autoSelector.getSelected();
    }


    public void setVisionPose(Pose2d pose) {
        visionField.setRobotPose(pose);
    }

    public void periodic() {
        field.setRobotPose(RobotContainer.s_Swerve.getPose()); // updates the robot's estimated position on the field
        SwerveModulePosition[] positions = RobotContainer.s_Swerve.getPositions();
        for(int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("swerveMod" + i, positions[i].angle.getDegrees());
        }
        SmartDashboard.putNumber("Game Timer", DriverStation.getMatchTime());
        // SmartDashboard.putNumber("swerveMod0", positions[0].angle.getDegrees());
    }

    public Command getCommand() {return new DataBoardCommand().ignoringDisable(true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);}
    private class DataBoardCommand extends CommandBase {
        @Override
        public void initialize() {
            init();
        }
        @Override
        public void execute() {
            periodic();
        }
    }
}

