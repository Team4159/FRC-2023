package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WheeledIntakeConstants.WheeledIntakeState;
import frc.robot.Constants.JoystickConstants.PrimaryDrive;
import frc.robot.Constants.JoystickConstants.PrimaryLeft;
import frc.robot.Constants.JoystickConstants.Secondary;
import frc.robot.commands.StraightAutobalance;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.CascadingArm;
import frc.robot.subsystems.RotatingArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WheeledIntake;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.StateController.GameElementState;
import frc.robot.subsystems.StateController.PositionState;

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
    //private final JoystickButton lockedMode = new JoystickButton(primaryDrive, PrimaryDrive.lockedMode); // locks the wheels like an X so it's harder to be pushed around
    private final JoystickButton toggleRotateLock = new JoystickButton(primaryDrive, PrimaryDrive.toggleRotateLock);
    private final JoystickButton rotateLockClockwise = new JoystickButton(primaryDrive, PrimaryDrive.rotateLockClockwise);
    private final JoystickButton rotateLockCounterclockwise = new JoystickButton(primaryDrive, PrimaryDrive.rotateLockCounterclockwise);

    private final JoystickButton aimbot = new JoystickButton(primaryLeft, PrimaryLeft.aimbot); // lines up for scoring automatically
    private final JoystickButton forceAcceptVision = new JoystickButton(primaryLeft, PrimaryLeft.forceAcceptVision); // lines up for scoring automatically

    /* Secondary buttons */
    private final JoystickButton outtake = new JoystickButton(secondary, Secondary.outtake);
    private final JoystickButton intake = new JoystickButton(secondary, Secondary.intake);

    private final JoystickButton cube = new JoystickButton(secondary, Secondary.cube);
    private final JoystickButton cone = new JoystickButton(secondary, Secondary.cone);
    
    private final JoystickButton high = new JoystickButton(secondary, Secondary.high);
    private final JoystickButton mid = new JoystickButton(secondary, Secondary.mid);
    private final JoystickButton low = new JoystickButton(secondary, Secondary.low);
    
    private final JoystickButton groundIntake = new JoystickButton(secondary, Secondary.groundIntake);
    private final JoystickButton singleIntake = new JoystickButton(secondary, Secondary.singleIntake);
    private final JoystickButton doubleIntake = new JoystickButton(secondary, Secondary.doubleIntake);
    
    private final JoystickButton tucked = new JoystickButton(secondary, Secondary.tucked);

    // private final JoystickButton togglePincerArm = new JoystickButton(secondary, Secondary.togglePincerArm);
    /*private final JoystickButton setRotateTucked = new JoystickButton(secondary, 10);
    private final JoystickButton setRotateIntakeOne = new JoystickButton(secondary, 8);
    private final JoystickButton setRotateLow = new JoystickButton(secondary, 7);
    private final JoystickButton setRotateMid = new JoystickButton(secondary, 6);
    private final JoystickButton setRotateHigh = new JoystickButton(secondary, 5);

    private final JoystickButton setCascadeTucked = new JoystickButton(secondary, 16);
    private final JoystickButton setCascadeIntakeOne = new JoystickButton(secondary, 14);
    private final JoystickButton setCascadeLow = new JoystickButton(secondary, 13);
    private final JoystickButton setCascadeMid = new JoystickButton(secondary, 12);

    private final JoystickButton setLEDYellow = new JoystickButton(secondary, 9);
    private final JoystickButton setLEDPurple = new JoystickButton(secondary, 11);
    private final JoystickButton setLEDPride = new JoystickButton(secondary, 15);*/



    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Vision vision = new Vision();
    public static final DataBoard dataBoard = new DataBoard();
    public static final CascadingArm cascadingArm = new CascadingArm();
    public static final RotatingArm rotatingArm = new RotatingArm();
    public static final Wrist wrist = new Wrist();
    public static final WheeledIntake wheeledIntake = new WheeledIntake();
    public static final LED led = new LED();
    
    public static final StateController stateController = new StateController();


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
                () -> false
            )
        );

        // Configure the event map for auto
        configureEventMap();

        // Configure the button bindings
        configureButtonBindings();

        dataBoard.getCommand().schedule();
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

        /*lockedMode
            .onTrue(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.LOCKED)))
            .onFalse(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.NORMAL)))
        ;*/

        toggleRotateLock
            .onTrue(new InstantCommand(() -> s_Swerve.lockRotateClosest()))
            .onFalse(new InstantCommand(() -> s_Swerve.disableLockRotate()))
        ;

        rotateLockClockwise.onTrue(new InstantCommand(() -> s_Swerve.lockRotateClockwise()));
        rotateLockCounterclockwise.onTrue(new InstantCommand(() -> s_Swerve.lockRotateCounterclockwise()));

        aimbot
            .onTrue(new InstantCommand(() -> {
                led.setState(LEDState.RAINBOWCYCLE);
                s_Swerve.setSwerveState(Swerve.TeleopState.AIMBOT);
            }))
            .whileTrue(new ProxyCommand(() -> s_Swerve.aimbot()))
            .onFalse(new InstantCommand(() -> {
                s_Swerve.setSwerveState(Swerve.TeleopState.NORMAL);
                led.setState(LEDState.BLACK);
            }))
        ;

        forceAcceptVision.onTrue(new InstantCommand(() -> s_Swerve.forceAcceptNextVision()));
        
        // setLEDYellow.and(setLEDPurple.negate())
        //     .onTrue(new InstantCommand(() -> led.setState(LEDState.YELLOW)))
        //     .onFalse(new InstantCommand(() ->led.setState(LEDState.RAINBOW)));
        // setLEDPurple.and(setLEDYellow.negate())
        //     .onTrue(new InstantCommand(() -> led.setState(LEDState.PURPLE)))
        //     .onFalse(new InstantCommand(() ->led.setState(LEDState.RAINBOW)));
        // setLEDPride
        //     .onTrue(new InstantCommand(() -> led.setState(LEDState.RAINBOWCYCLE)));
            // .whileTrue(new SequentialCommandGroup(
            //     new InstantCommand(() -> led.setState(LEDState.GAY)),
            //     new WaitCommand(4),
            //     new InstantCommand(() -> led.setState(LEDState.LESBIAN)),
            //     new WaitCommand(4),
            //     new InstantCommand(() -> led.setState(LEDState.BI)),
            //     new WaitCommand(4),
            //     new InstantCommand(() -> led.setState(LEDState.NONBINARY)),
            //     new WaitCommand(4),
            //     new InstantCommand(() -> led.setState(LEDState.GENDERFLUID)),
            //     new WaitCommand(4),
            //     new InstantCommand(() -> led.setState(LEDState.TRANS)),
            //     new WaitCommand(4)
            // ));

        /*setRotateTucked.onTrue(new InstantCommand(() -> rotatingArm.setArmState(RotateState.TUCKED)));
        setRotateIntakeOne.onTrue(new InstantCommand(() -> rotatingArm.setArmState(RotateState.INTAKING)));
        setRotateLow.onTrue(new InstantCommand(() -> rotatingArm.setArmState(RotateState.LOW)));
        setRotateMid.onTrue(new InstantCommand(() -> rotatingArm.setArmState(RotateState.MID)));
        setRotateHigh.onTrue(new InstantCommand(() -> rotatingArm.setArmState(RotateState.HIGH)));
        
        setCascadeTucked.onTrue(new InstantCommand(() -> cascadingArm.setArmState(CascadeState.TUCKED)));
        setCascadeIntakeOne.onTrue(new InstantCommand(() -> cascadingArm.setArmState(CascadeState.INTAKING)));
        setCascadeLow.onTrue(new InstantCommand(() -> cascadingArm.setArmState(CascadeState.SCORING1)));
        setCascadeMid.onTrue(new InstantCommand(() -> cascadingArm.setArmState(CascadeState.SCORING2)));*/
        
        intake.onTrue(new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.INTAKE)))
            .onFalse(new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL))
        );
        outtake.onTrue(new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)))
            .onFalse(new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL))
        );

        cube.onTrue(stateController.setGameElementState(GameElementState.CUBE));
        cone.onTrue(stateController.setGameElementState(GameElementState.CONE));

        high.onTrue(stateController.setPositionState(PositionState.HIG_SCORE));
        mid.onTrue(stateController.setPositionState(PositionState.MID_SCORE));
        low.onTrue(stateController.setPositionState(PositionState.LOW_SCORE));

        groundIntake.onTrue(stateController.setPositionState(PositionState.GROUND_INTAKING));
        singleIntake.onTrue(stateController.setPositionState(PositionState.SINGLE_SUBSTATION));
        doubleIntake.onTrue(stateController.setPositionState(PositionState.DOUBLE_SUBSTATION));
        
        tucked.onTrue(stateController.setPositionState(PositionState.TUCKED));
        
            
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

        eventMap.put("GroundIntakeCube", stateController.autoGroundIntake(GameElementState.CUBE));
        eventMap.put("GroundIntakeCube", stateController.autoGroundIntake(GameElementState.CONE));

        eventMap.put("ScoreLowCube", stateController.autoScoreLow(GameElementState.CUBE));
        eventMap.put("ScoreMidCube", stateController.autoScoreMid(GameElementState.CUBE));
        eventMap.put("ScoreHighCube", stateController.autoScoreHigh(GameElementState.CUBE));

        eventMap.put("ScoreLowCone", stateController.autoScoreLow(GameElementState.CONE));
        eventMap.put("ScoreMidCone", stateController.autoScoreMid(GameElementState.CONE));
        eventMap.put("ScoreHighCone", stateController.autoScoreHigh(GameElementState.CONE));

        //eventMap.put("Autobalance", null);
        
        eventMap.put("LEDYellow", new InstantCommand(() -> led.setState(LEDState.YELLOW)));
        eventMap.put("LEDPurple", new InstantCommand(() -> led.setState(LEDState.PURPLE)));

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
        if(dataBoard.getAutoPos() == -1) return new PrintCommand("Auto Disabled");
        if(dataBoard.getAutoPos() == 0) return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(
                (DriverStation.getAlliance().equals(Alliance.Blue)) ? new Pose2d(new Translation2d(1.84, 2.75), Rotation2d.fromDegrees(0))
                : new Pose2d(new Translation2d(VisionConstants.fieldWidth-1.84, 2.75), Rotation2d.fromDegrees(180))
            )),
            new StraightAutobalance(s_Swerve)
        );
        final var traj = autos.get(DriverStation.getAlliance())
                .get(dataBoard.getAutoPos() - 1)
                .get(dataBoard.getAutoMode());
        if (traj == null) return null;
        return new SequentialCommandGroup(
            new InstantCommand(() -> led.setState(LEDState.RAINBOWCYCLE)),
            autoBuilder.fullAuto(traj),
            new InstantCommand(() -> led.setState(LEDState.BLACK))
        );
    }
}
