package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    private final UpdateVision updateVision;
    
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = Joystick.AxisType.kY.value;
    private final int strafeAxis = Joystick.AxisType.kX.value;
    private final int rotationAxis = Joystick.AxisType.kZ.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton alignRobot = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();

    /* Vision Commands */
    //private static final Command updateVision = new UpdateVision(s_Swerve);
    private static final Command swapToRetro = new SwapVisionPipeline(0);
    private static final Command swapToApril = new SwapVisionPipeline(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        //updateVision.repeatedly().schedule();
        updateVision = new UpdateVision(s_Swerve);

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
        alignRobot.debounce(5).onTrue(swapToRetro).onFalse(swapToApril);
    }

    public static enum AutoMode {Dock, Normal}
    /*private Map<DriverStation.Alliance, Map<Integer, Map<AutoMode, Command>>> autos = Map.of(
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
    );*/

    public Command getAutonomousCommand() {
        /*return autos.get(DriverStation.getAlliance())
            .get((int)NetworkTableInstance.getDefault().getTable("FMSInfo").getValue("StationNumber").getInteger())
            .get(dataBoard.getAutoMode());*/
        return new B1S1(s_Swerve);
    }
}
