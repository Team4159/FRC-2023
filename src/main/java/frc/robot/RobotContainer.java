package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.JoystickConstants.*;
import frc.robot.commands.AimbotSwerve;
import frc.robot.commands.RunPincerArm;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.PincerArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.autos.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {    
    /* Controllers */
    private final Joystick primaryDrive = new Joystick(PrimaryDrive.drivePort); // translational movement
    private final Joystick primaryTurn = new Joystick(PrimaryTurn.turnPort); // rotational movement

    @SuppressWarnings("unused")
    private final Joystick secondary = new Joystick(Secondary.secondaryPort); // other robot controls

    /* Drive Controls */
    private final int translationAxis = Joystick.AxisType.kY.value;
    private final int strafeAxis = Joystick.AxisType.kX.value;

    private final int rotationAxis = Joystick.AxisType.kX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(primaryDrive, PrimaryDrive.zeroGyro); // zeroes the gyro based on the current robot rotation
    private final JoystickButton lockedMode = new JoystickButton(primaryDrive, PrimaryDrive.lockedMode); // locks the wheels like an X so it's harder to be pushed around

    private final JoystickButton aimbot = new JoystickButton(primaryTurn, PrimaryTurn.aimbot); // lines up for scoring automatically

    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); TODO: is this even necessary?

    private final JoystickButton runPincerArmButton = new JoystickButton(secondary, 1);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Vision vision = new Vision();
    public static final DataBoard dataBoard = new DataBoard();
    private final PincerArm pincerArm = new PincerArm();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -primaryDrive.getRawAxis(translationAxis), 
                () -> -primaryDrive.getRawAxis(strafeAxis), 
                () -> -primaryTurn.getRawAxis(rotationAxis), 
                () -> false // TODO: robotCentric button?
            )
        );

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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        lockedMode
            .onTrue(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.LOCKED)))
            .onFalse(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.NORMAL)))
        ;

        aimbot
            .onTrue(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.AIMBOT)))
            .whileTrue(new AimbotSwerve(s_Swerve, FieldRegion.lookup(s_Swerve.getPose()))) // may want to disable this when too far from goal
            .onFalse(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.NORMAL)))
        ;
        
        //alignRobot.debounce(5).onTrue(swapToRetro).onFalse(swapToApril);
        runPincerArmButton.onTrue(new RunPincerArm(pincerArm));
    }

    public static enum AutoMode {Dock, Normal}
    private Map<DriverStation.Alliance, Map<Integer, Map<AutoMode, Command>>> autos = Map.of(
        DriverStation.Alliance.Red, Map.<Integer, Map<AutoMode, Command>>of( // Red Alliance
            0, Map.<AutoMode, Command>of( // Station 1
                // AutoMode.Dock, null, // Dock
                AutoMode.Normal, new B1(s_Swerve) // Don't Dock
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

    public Command getAutonomousCommand() {
        return autos.get(DriverStation.getAlliance())
            .get((int)NetworkTableInstance.getDefault().getTable("FMSInfo").getValue("StationNumber").getInteger())
            .get(dataBoard.getAutoMode());
    }
}
