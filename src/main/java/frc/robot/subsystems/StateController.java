package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.CascadingArmConstants.CascadeState;
import frc.robot.Constants.RotatingArmConstants.RotateState;
import frc.robot.Constants.WheeledIntakeConstants.WheeledIntakeState;
import frc.robot.Constants.WristConstants.WristState;
import frc.robot.subsystems.LED.LEDState;

public class StateController extends SubsystemBase {
    private LED led;
    private RotatingArm rotatingArm;
    private CascadingArm cascadingArm;
    private Wrist wrist;
    private WheeledIntake wheeledIntake;

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
        wheeledIntake = RobotContainer.wheeledIntake;


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
                        return retractRotateExtend(CascadeState.TUCKED, RotateState.TUCKED, WristState.TUCKED, noRetract);
                    case GROUND_INTAKING:
                        led.setState(LEDState.YELLOW);
                        return retractRotateExtend(null, null, null, noRetract);
                    case SINGLE_SUBSTATION:
                        led.setState(LEDState.YELLOW);
                        return retractRotateExtend(null, null, null, noRetract);
                    case DOUBLE_SUBSTATION:
                        led.setState(LEDState.YELLOW);
                        return retractRotateExtend(null, null, null, noRetract);
                    case LOW_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(null, null, null, noRetract);
                    case MID_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(null, null, null, noRetract);
                    case HIG_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(null, null, null, noRetract);
                }
            case CUBE:
                switch (positionState) {
                    case TUCKED:
                        led.setState(LEDState.PURPLE);
                        return retractRotateExtend(CascadeState.TUCKED, RotateState.TUCKED, WristState.TUCKED, noRetract);
                    case GROUND_INTAKING:
                        led.setState(LEDState.PURPLE);
                        return retractRotateExtend(null, null, null, noRetract);
                    case SINGLE_SUBSTATION:
                        led.setState(LEDState.PURPLE);
                        return retractRotateExtend(null, null, null, noRetract);
                    case DOUBLE_SUBSTATION:
                        led.setState(LEDState.PURPLE);
                        return retractRotateExtend(null, null, null, noRetract);
                    case LOW_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(null, null, null, noRetract);
                    case MID_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(null, null, null, noRetract);
                    case HIG_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(null, null, null, noRetract);
                }
        }
        return new PrintCommand("State Controller broken :(");
    }

    public Command retractRotateExtend(CascadeState cascadeState, RotateState rotateState, WristState wristState, boolean noRetract) { // trust the process :(
        if (noRetract) return new SequentialCommandGroup(
            new ParallelCommandGroup(
                (new InstantCommand(() -> rotatingArm.setArmState(rotateState)).repeatedly()).until(() -> rotatingArm.atDesiredSetPoint()),
                (new InstantCommand(() -> wrist.setArmState(wristState)).repeatedly()).until(() -> wrist.atDesiredSetPoint())
            ),
            (new InstantCommand(() -> cascadingArm.setArmState(cascadeState)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint())
        );
        
        return new SequentialCommandGroup(
            (new InstantCommand(() -> cascadingArm.setArmState(CascadeState.TUCKED)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint()),
            new ParallelCommandGroup(
                (new InstantCommand(() -> rotatingArm.setArmState(rotateState)).repeatedly()).until(() -> rotatingArm.atDesiredSetPoint()),
                (new InstantCommand(() -> wrist.setArmState(wristState)).repeatedly()).until(() -> wrist.atDesiredSetPoint())
            ),
            ((new InstantCommand(() -> cascadingArm.setArmState(cascadeState)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint()))
        );
    }

    public Command autoScoreLow(GameElementState gameElementState) {
        return new SequentialCommandGroup(
            updateLocalStates(false),
            setGameElementState(gameElementState),
            setPositionState(PositionState.LOW_SCORE),
            new WaitCommand(0.1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.3),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            setPositionState(PositionState.TUCKED)
        );
    }

    public Command autoScoreMid(GameElementState gameElementState) {
        return new SequentialCommandGroup(
            updateLocalStates(false),
            setGameElementState(gameElementState),
            setPositionState(PositionState.MID_SCORE),
            new WaitCommand(0.1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.3),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            setPositionState(PositionState.TUCKED)
        );
    }

    public Command autoScoreHigh(GameElementState gameElementState) {
        return new SequentialCommandGroup(
            updateLocalStates(false),
            setGameElementState(gameElementState),
            setPositionState(PositionState.HIG_SCORE),
            new WaitCommand(0.1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.3),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            setPositionState(PositionState.TUCKED)
        );
    }

    public Command autoGroundIntake(GameElementState gameElementState) {
        return new SequentialCommandGroup(
            updateLocalStates(false),
            setGameElementState(gameElementState),
            setPositionState(PositionState.GROUND_INTAKING),
            new WaitCommand(0.1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.INTAKE)),
            new WaitCommand(0.3),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            setPositionState(PositionState.TUCKED)
        );
    }
}
