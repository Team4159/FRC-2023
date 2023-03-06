package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CascadingArmConstants.CascadeState;
import frc.robot.Constants.RotatingArmConstants.RotateState;
import frc.robot.Constants.WristConstants.WristState;
import frc.robot.subsystems.LED.LEDState;

public class StateController extends SubsystemBase {
    private LED led;
    private RotatingArm rotatingArm;
    private CascadingArm cascadingArm;
    private Wrist wrist;

    private GameElementState gameElementState;
    private PositionState positionState;

    public static enum GameElementState {
        CUBE,
        CONE
    }

    public static enum PositionState {
        TUCKED,
        GROUND_INTAKING,
        SINGLE_SUBSTATION,
        DOUBLE_SUBSTATION,
        LOW_SCORE,
        MID_SCORE,
        HIG_SCORE
    }

    public StateController() {
        led = RobotContainer.led;
        rotatingArm = RobotContainer.rotatingArm;
        cascadingArm = RobotContainer.cascadingArm;
        wrist = RobotContainer.wrist;


        gameElementState = GameElementState.CUBE;
        positionState = PositionState.TUCKED;
    }

    public Command setGameElementState(GameElementState gameElementState) {
        this.gameElementState = gameElementState;
        return updateLocalStates(true);
    }

    public Command setPositionState(PositionState positionState) {
        this.positionState = positionState;
        return updateLocalStates(false);
    }

    public Command updateLocalStates(boolean noRetract) {
        switch (gameElementState) {
            case CONE:
                switch (positionState) {
                    case TUCKED:
                        led.setState(LEDState.YELLOW);
                        return RetractRotateExtend(CascadeState.TUCKED, RotateState.TUCKED, WristState.TUCKED, noRetract);
                    case GROUND_INTAKING:
                        led.setState(LEDState.YELLOW);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case SINGLE_SUBSTATION:
                        led.setState(LEDState.YELLOW);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case DOUBLE_SUBSTATION:
                        led.setState(LEDState.YELLOW);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case LOW_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case MID_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case HIG_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return RetractRotateExtend(null, null, null, noRetract);
                }
            case CUBE:
                switch (positionState) {
                    case TUCKED:
                        led.setState(LEDState.PURPLE);
                        return RetractRotateExtend(CascadeState.TUCKED, RotateState.TUCKED, WristState.TUCKED, noRetract);
                    case GROUND_INTAKING:
                        led.setState(LEDState.PURPLE);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case SINGLE_SUBSTATION:
                        led.setState(LEDState.PURPLE);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case DOUBLE_SUBSTATION:
                        led.setState(LEDState.PURPLE);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case LOW_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case MID_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return RetractRotateExtend(null, null, null, noRetract);
                    case HIG_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return RetractRotateExtend(null, null, null, noRetract);
                }
        }
        return new PrintCommand("State Controller broken :(");
    }

    public Command RetractRotateExtend(CascadeState cascadeState, RotateState rotateState, WristState wristState, boolean noRetract) { // trust the process :(
        if (noRetract) return new SequentialCommandGroup(
            (((new InstantCommand(() -> rotatingArm.setArmState(rotateState)).repeatedly()).until(() -> rotatingArm.atDesiredSetPoint()))
            .alongWith((new InstantCommand(() -> wrist.setArmState(wristState)).repeatedly()).until(() -> wrist.atDesiredSetPoint())))
            .andThen((new InstantCommand(() -> cascadingArm.setArmState(cascadeState)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint()))
        );
        
        return new SequentialCommandGroup(
            ((new InstantCommand(() -> cascadingArm.setArmState(CascadeState.TUCKED)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint()))
            .andThen(((new InstantCommand(() -> rotatingArm.setArmState(rotateState)).repeatedly()).until(() -> rotatingArm.atDesiredSetPoint()))
            .alongWith((new InstantCommand(() -> wrist.setArmState(wristState)).repeatedly()).until(() -> wrist.atDesiredSetPoint())))
            .andThen((new InstantCommand(() -> cascadingArm.setArmState(cascadeState)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint()))
        );
    }
}
