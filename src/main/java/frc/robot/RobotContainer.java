package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants.PrimaryDrive;
import frc.robot.Constants.JoystickConstants.PrimaryLeft;
import frc.robot.Constants.JoystickConstants.Secondary;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.CascadingArm;
import frc.robot.subsystems.PincerArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CascadingArm.ArmState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {    
    /* Controllers */
    private final Joystick primaryDrive = new Joystick(PrimaryDrive.drivePort); // translational movement
    private final Joystick primaryLeft = new Joystick(PrimaryLeft.leftPort); // rotational movement

    private final Joystick secondary = new Joystick(Secondary.secondaryPort); // other robot controls

    /* Drive Controls */
    private final int translationAxis = Joystick.AxisType.kY.value;
    private final int strafeAxis = Joystick.AxisType.kX.value;

    private final int rotationAxis = Joystick.AxisType.kZ.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(primaryDrive, PrimaryDrive.zeroGyro); // zeroes the gyro based on the current robot rotation
    private final JoystickButton lockedMode = new JoystickButton(primaryDrive, PrimaryDrive.lockedMode); // locks the wheels like an X so it's harder to be pushed around

    private final JoystickButton aimbot = new JoystickButton(primaryLeft, PrimaryLeft.aimbot); // lines up for scoring automatically
    private final JoystickButton forceAcceptVision = new JoystickButton(primaryLeft, PrimaryLeft.forceAcceptVision); // lines up for scoring automatically

    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); TODO: is this even necessary?

    private final JoystickButton togglePincerArm = new JoystickButton(secondary, Secondary.togglePincerArm);


    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Vision vision = new Vision();
    public static final DataBoard dataBoard = new DataBoard();
    public static final CascadingArm cascadingArm = new CascadingArm();
    public static final PincerArm pincerArm = new PincerArm();

    public static final Map<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        s_Swerve::getPose, // Pose2d supplier
        s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -primaryDrive.getRawAxis(translationAxis), 
                () -> -primaryDrive.getRawAxis(strafeAxis), 
                () -> -primaryDrive.getRawAxis(rotationAxis), 
                () -> false // TODO: robotCentric button?
            )
        );

        // Configure the event map for auto
        configureEventMap();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyroOffset()));

        lockedMode
            .onTrue(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.LOCKED)))
            .onFalse(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.NORMAL)))
        ;

        aimbot
            .onTrue(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.AIMBOT)))
            .whileTrue(s_Swerve.aimbot())
            .onFalse(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.NORMAL)))
        ;

        forceAcceptVision.onTrue(new InstantCommand(() -> s_Swerve.forceAcceptNextVision()));
        
        //alignRobot.debounce(5).onTrue(swapToRetro).onFalse(swapToApril);
        togglePincerArm.onTrue(new InstantCommand(() -> pincerArm.togglePincerArm()));
        
    }

    public void teleopInit() {
        s_Swerve.setGyroOffset((DriverStation.getAlliance().equals(Alliance.Red)) // sets the user gyro offset depending on if the driver is on the red or blue alliance
            ? Rotation2d.fromDegrees(180)
            : Rotation2d.fromDegrees(0)
        );
    }

    private void configureEventMap() {
        eventMap.put("Print1", new PrintCommand("Print 1"));
        eventMap.put("Print2", new PrintCommand("Print 2"));
        eventMap.put("Print3", new PrintCommand("Print 3"));
        eventMap.put("RotateArmDown", null); // TODO: finish these commands
        eventMap.put("RotateArmUp", null);
        eventMap.put("RotateArmIntake", null);
        eventMap.put("CascadeIntake", new InstantCommand(() -> cascadingArm.setArmState(ArmState.INTAKING)));
        eventMap.put("CascadeIn", null); // TODO: tucked in cascade arm
        eventMap.put("CascadeOne", new InstantCommand(() -> cascadingArm.setArmState(ArmState.SCORING1)));
        eventMap.put("CascadeTwo", new InstantCommand(() -> cascadingArm.setArmState(ArmState.SCORING2)));
        eventMap.put("CascadeThree", new InstantCommand(() -> cascadingArm.setArmState(ArmState.SCORING3)));
        eventMap.put("PincerIn", new InstantCommand(() -> pincerArm.setPincerArm(Value.kForward)));
        eventMap.put("PincerOut", new InstantCommand(() -> pincerArm.setPincerArm(Value.kReverse)));
        // eventMap.put("FullIntake", new SequentialCommandGroup(null));
        // eventMap.put("ScoreLow", new SequentialCommandGroup(null));
        // eventMap.put("ScoreMid", new SequentialCommandGroup(null));
        // eventMap.put("ScoreHigh", new SequentialCommandGroup(null));
        // eventMap.put("Autobalance", null);
        // eventMap.put("LEDYellow", null);
        // eventMap.put("LEDPurple", null);
        eventMap.put("Wait0.1", new WaitCommand(0.1));
        eventMap.put("Wait0.5", new WaitCommand(0.5));
        eventMap.put("Wait1", new WaitCommand(1));
        eventMap.put("Wait2", new WaitCommand(2));
        eventMap.put("Wait5", new WaitCommand(5));
    }

    public static enum AutoMode {Dock, Normal}
    private Map<DriverStation.Alliance, Map<Integer, Map<AutoMode, List<PathPlannerTrajectory>>>> autos = Map.of(
        DriverStation.Alliance.Red, Map.<Integer, Map<AutoMode, List<PathPlannerTrajectory>>>of( // Red Alliance
            0, Map.<AutoMode, List<PathPlannerTrajectory>>of( // Station 1
                // AutoMode.Dock, null, // Dock
                AutoMode.Normal, loadPathGroup("Bsimple1") // Don't Dock
            )//,
            // 1, Map.<AutoMode, Command>of( // Station 2
            //     AutoMode.Dock, null,
            //     AutoMode.Normal, null
            // ),
            // 2, Map.<AutoMode, Command>of( // Station 3
            //     AutoMode.Dock, null,
            //     AutoMode.Normal, null
            // )
        )//,
        // DriverStation.Alliance.Blue, Map.<Integer, Map<AutoMode, Command>>of( // Blue Alliance
        //     0, Map.<AutoMode, Command>of( // Station 1
        //         AutoMode.Dock, null, // Dock
        //         AutoMode.Normal, null // Don't Dock
        //     ),
        //     1, Map.<AutoMode, Command>of( // Station 2
        //         AutoMode.Dock, null,
        //         AutoMode.Normal, null
        //     ),
        //     2, Map.<AutoMode, Command>of( // Station 3
        //         AutoMode.Dock, null,
        //         AutoMode.Normal, null
        //     )
        // )
    );

    private List<PathPlannerTrajectory> loadPathGroup(String s) {
        return PathPlanner.loadPathGroup(s, Constants.AutoConstants.kPathConstraints);
    }

    public Command getAutonomousCommand() {
        return autoBuilder.fullAuto(autos.get(DriverStation.getAlliance())
            .get((int)NetworkTableInstance.getDefault().getTable("FMSInfo").getValue("StationNumber").getInteger())
            .get(dataBoard.getAutoMode()));
    }
}
