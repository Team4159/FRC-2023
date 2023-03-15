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

    /*@Override
    public void periodic() {
        System.out.println("Rotate: " + rotatingArm.getEncoderPosition());
        System.out.println("Cascade: " + cascadingArm.getEncoderPosition());
        System.out.println("Wrist: " + wrist.getEncoderPosition());
    }*/

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
        cascadingArm.resetIntegrator();
        rotatingArm.resetIntegrator();
        return updateLocalStates(true);
    }

    public Command setPositionState(PositionState positionState) {
        this.positionState = positionState;
        cascadingArm.resetIntegrator();
        rotatingArm.resetIntegrator();
        return updateLocalStates(false);
    }

    public Command updateLocalStates(boolean noRetract) {
        switch (gameElementState) {
            case CONE:
                switch (positionState) {
                    case TUCKED:
                        led.setState(LEDState.YELLOW);
                        return retractRotateExtend(CascadeState.TUCKED_CONE, RotateState.TUCKED_CONE, WristState.TUCKED_CONE, noRetract);
                    case GROUND_INTAKING:
                        led.setState(LEDState.YELLOW);
                        return retractRotateExtend(CascadeState.GROUND_INTAKE_CONE, RotateState.GROUND_INTAKE_CONE, WristState.GROUND_INTAKE_CONE, noRetract);
                    case SINGLE_SUBSTATION:
                        led.setState(LEDState.YELLOW);
                        return retractRotateExtend(null, null, null, noRetract);
                    case DOUBLE_SUBSTATION:
                        led.setState(LEDState.YELLOW);
                        return retractRotateExtend(null, null, null, noRetract);
                    case LOW_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(CascadeState.LOW_CONE, RotateState.LOW_CONE, WristState.LOW_CONE, noRetract);
                    case MID_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(CascadeState.MID_CONE, RotateState.MID_CONE, WristState.MID_CONE, noRetract);
                    case HIG_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(CascadeState.HIGH_CONE, RotateState.HIGH_CONE, WristState.HIGH_CONE, noRetract);
                }
            case CUBE:
                switch (positionState) {
                    case TUCKED:
                        led.setState(LEDState.PURPLE);
                        return retractRotateExtend(CascadeState.TUCKED_CUBE, RotateState.TUCKED_CUBE, WristState.TUCKED_CUBE, noRetract);
                    case GROUND_INTAKING:
                        led.setState(LEDState.PURPLE);
                        return retractRotateExtend(CascadeState.GROUND_INTAKE_CUBE, RotateState.GROUND_INTAKE_CUBE, WristState.GROUND_INTAKE_CUBE, noRetract);
                    case SINGLE_SUBSTATION:
                        led.setState(LEDState.PURPLE);
                        return retractRotateExtend(null, null, null, noRetract);
                    case DOUBLE_SUBSTATION:
                        led.setState(LEDState.PURPLE);
                        return retractRotateExtend(null, null, null, noRetract);
                    case LOW_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(CascadeState.LOW_CUBE, RotateState.LOW_CUBE, WristState.LOW_CUBE, noRetract);
                    case MID_SCORE:
                        led.setState(LEDState.RAINBOW);
                        System.out.println("MID!");
                        return retractRotateExtend(CascadeState.MID_CUBE, RotateState.MID_CUBE, WristState.MID_CUBE, noRetract);
                    case HIG_SCORE:
                        led.setState(LEDState.RAINBOW);
                        return retractRotateExtend(CascadeState.HIGH_CUBE, RotateState.HIGH_CUBE, WristState.HIGH_CUBE, noRetract);
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
            (new InstantCommand(() -> cascadingArm.setArmState((gameElementState == GameElementState.CONE) ? CascadeState.TUCKED_CONE : CascadeState.TUCKED_CUBE)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint()),
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
