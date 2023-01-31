package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.JoystickConstants.*;
import frc.robot.commands.AimbotSwerve;
import frc.robot.commands.SwapVisionPipeline;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.UpdateVision;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final DataBoard dataBoard = new DataBoard();
    
    /* Controllers */
    private final Joystick primaryDrive = new Joystick(PrimaryDrive.drivePort);
    private final Joystick primaryTurn = new Joystick(PrimaryTurn.turnPort);

    private final Joystick secondary = new Joystick(Secondary.secondaryPort);

    /* Drive Controls */
    private final int translationAxis = Joystick.AxisType.kY.value;
    private final int strafeAxis = Joystick.AxisType.kX.value;

    private final int rotationAxis = Joystick.AxisType.kX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(primaryDrive, PrimaryDrive.zeroGyro);
    private final JoystickButton lockedMode = new JoystickButton(primaryDrive, PrimaryDrive.lockedMode);

    private final JoystickButton aimbot = new JoystickButton(primaryTurn, PrimaryTurn.aimbot);


    //private final JoystickButton alignRobot = new JoystickButton(primaryDrive, XboxController.Axis.kLeftTrigger.value); TODO: fix

    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); TODO: is this even necessary?

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();

    /* Vision Commands */
    //private static final Command updateVision = new UpdateVision(s_Swerve);
    private static final Command swapToRetro = new SwapVisionPipeline(0);
    private static final Command swapToApril = new SwapVisionPipeline(1);
    
    private final Command updateVision = new UpdateVision(s_Swerve, dataBoard);

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
        updateVision.repeatedly().schedule();

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
            .whileTrue(new AimbotSwerve(s_Swerve, s_Swerve.getTargetPose()))
            .onFalse(new InstantCommand(() -> s_Swerve.setSwerveState(Swerve.TeleopState.NORMAL)))
        ;
        
        //alignRobot.debounce(5).onTrue(swapToRetro).onFalse(swapToApril);
    }

    public static enum AutoMode {Dock, Normal}
    private Map<DriverStation.Alliance, Map<Integer, Map<AutoMode, Command>>> autos = Map.of(
        DriverStation.Alliance.Red, Map.<Integer, Map<AutoMode, Command>>of( // Red Alliance
            0, Map.<AutoMode, Command>of( // Station 1
                AutoMode.Dock, null, // Dock
                AutoMode.Normal, new B1S1(s_Swerve) // Don't Dock
            ),
            1, Map.<AutoMode, Command>of( // Station 2
                AutoMode.Dock, null,
                AutoMode.Normal, null
            ),
            2, Map.<AutoMode, Command>of( // Station 3
                AutoMode.Dock, null,
                AutoMode.Normal, null
            )
        ),
        DriverStation.Alliance.Blue, Map.<Integer, Map<AutoMode, Command>>of( // Blue Alliance
            0, Map.<AutoMode, Command>of( // Station 1
                AutoMode.Dock, null, // Dock
                AutoMode.Normal, null // Don't Dock
            ),
            1, Map.<AutoMode, Command>of( // Station 2
                AutoMode.Dock, null,
                AutoMode.Normal, null
            ),
            2, Map.<AutoMode, Command>of( // Station 3
                AutoMode.Dock, null,
                AutoMode.Normal, null
            )
        )
    );

    public Command getAutonomousCommand() {
        /*return autos.get(DriverStation.getAlliance())
            .get((int)NetworkTableInstance.getDefault().getTable("FMSInfo").getValue("StationNumber").getInteger())
            .get(dataBoard.getAutoMode());*/
        System.out.println(DriverStation.getAlliance().toString());
        System.out.println(NetworkTableInstance.getDefault().getTable("FMSInfo").getValue("StationNumber").getInteger());
        System.out.println(dataBoard.getAutoMode());
        return new B1S1(s_Swerve);
    }
}
