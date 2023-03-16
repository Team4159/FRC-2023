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
    private boolean forceScore;

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
        forceScore = false;
    }

    @Override
    public void periodic() {
    }

    public void setGameElementState(GameElementState gameElementState) {
        this.gameElementState = gameElementState;
        cascadingArm.resetIntegrator();
        rotatingArm.resetIntegrator();
        updateLocalStates();
    }

    public void setPositionState(PositionState positionState) {
        if (this.positionState == PositionState.MID_SCORE && (positionState == PositionState.GROUND_INTAKING || positionState == PositionState.TUCKED)) {
            this.positionState = PositionState.LOW_SCORE;
        }
        else if ((this.positionState == PositionState.HIG_SCORE || this.positionState == PositionState.DOUBLE_SUBSTATION) && (positionState == PositionState.GROUND_INTAKING || positionState == PositionState.LOW_SCORE || positionState == PositionState.TUCKED)) {
            this.positionState = PositionState.MID_SCORE;
        } else {
            this.positionState = positionState;
        }

        if (this.positionState == PositionState.TUCKED || this.positionState == PositionState.LOW_SCORE || this.positionState == PositionState.GROUND_INTAKING) {
            setForceScore(false);
        }
        cascadingArm.resetIntegrator();
        rotatingArm.resetIntegrator();
        updateLocalStates();
    }

    public void toggleForceScore() {
        forceScore = !forceScore;
        updateLocalStates();
    }

    public void setForceScore(boolean forceScore) {
        this.forceScore = forceScore;
        updateLocalStates();
    }

    public void setLocalStates(CascadeState cs, RotateState rs, WristState ws) {
        cascadingArm.setArmState(cs);
        rotatingArm.setArmState(rs);
        wrist.setArmState((forceScore) ? WristState.FORCE_SCORE : ws);
    }

    public void updateLocalStates() {
        if (gameElementState == GameElementState.CONE) {
            if (positionState == PositionState.TUCKED) {
                led.setState(LEDState.YELLOW);
                System.out.println("cone tuck");
                setLocalStates(CascadeState.TUCKED_CONE, RotateState.TUCKED_CONE, WristState.TUCKED_CONE);
            } else if (positionState == PositionState.GROUND_INTAKING) {
                led.setState(LEDState.YELLOW);
                System.out.println("cone ground");
                setLocalStates(CascadeState.GROUND_INTAKE_CONE, RotateState.GROUND_INTAKE_CONE, WristState.GROUND_INTAKE_CONE);
            } else if (positionState == PositionState.SINGLE_SUBSTATION) {
                led.setState(LEDState.YELLOW);
                System.out.println("cone single");
                setLocalStates(null, null, null);
            } else if (positionState == PositionState.DOUBLE_SUBSTATION) {
                led.setState(LEDState.YELLOW);
                System.out.println("cone double");
                setLocalStates(CascadeState.DOUBLE_INTAKE_CONE, RotateState.DOUBLE_INTAKE_CONE, WristState.DOUBLE_INTAKE_CONE);
            } else if (positionState == PositionState.LOW_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cone low");
                setLocalStates(CascadeState.LOW_CONE, RotateState.LOW_CONE, WristState.LOW_CONE);
            } else if (positionState == PositionState.MID_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cone mid");
                setLocalStates(CascadeState.MID_CONE, RotateState.MID_CONE, WristState.MID_CONE);
            } else if (positionState == PositionState.HIG_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cone high");
                setLocalStates(CascadeState.HIGH_CONE, RotateState.HIGH_CONE, WristState.HIGH_CONE);
            }
        } else if (gameElementState == GameElementState.CUBE) {
            if (positionState == PositionState.TUCKED) {
                led.setState(LEDState.PURPLE);
                System.out.println("cube tuck");
                setLocalStates(CascadeState.TUCKED_CUBE, RotateState.TUCKED_CUBE, WristState.TUCKED_CUBE);
            } else if (positionState == PositionState.GROUND_INTAKING) {
                led.setState(LEDState.PURPLE);
                System.out.println("cube ground");
                setLocalStates(CascadeState.GROUND_INTAKE_CUBE, RotateState.GROUND_INTAKE_CUBE, WristState.GROUND_INTAKE_CUBE);
            } else if (positionState == PositionState.SINGLE_SUBSTATION) {
                led.setState(LEDState.PURPLE);
                System.out.println("cube single");
                setLocalStates(null, null, null);
            } else if (positionState == PositionState.DOUBLE_SUBSTATION) {
                led.setState(LEDState.PURPLE);
                System.out.println("cube double");
                setLocalStates(CascadeState.DOUBLE_INTAKE_CUBE, RotateState.DOUBLE_INTAKE_CUBE, WristState.DOUBLE_INTAKE_CUBE);
            } else if (positionState == PositionState.LOW_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cube low");
                setLocalStates(CascadeState.LOW_CUBE, RotateState.LOW_CUBE, WristState.LOW_CUBE);
            } else if (positionState == PositionState.MID_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cube mid");
                setLocalStates(CascadeState.MID_CUBE, RotateState.MID_CUBE, WristState.MID_CUBE);
            } else if (positionState == PositionState.HIG_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cube high");
                setLocalStates(CascadeState.HIGH_CUBE, RotateState.HIGH_CUBE, WristState.HIGH_CUBE);
            }
        }
    }

    // public Command retractRotateExtend(CascadeState cascadeState, RotateState rotateState, WristState wristState, boolean noRetract) { // trust the process :(
    //     if (noRetract) return new SequentialCommandGroup(
    //         new ParallelCommandGroup(
    //             (new InstantCommand(() -> rotatingArm.setArmState(rotateState)).repeatedly()).until(() -> rotatingArm.atDesiredSetPoint()),
    //             (new InstantCommand(() -> wrist.setArmState(wristState)).repeatedly()).until(() -> wrist.atDesiredSetPoint())
    //         ),
    //         (new InstantCommand(() -> cascadingArm.setArmState(cascadeState)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint())
    //     );
        
    //     return new SequentialCommandGroup(
    //         (new InstantCommand(() -> cascadingArm.setArmState((gameElementState == GameElementState.CONE) ? CascadeState.TUCKED_CONE : CascadeState.TUCKED_CUBE)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint()),
    //         new ParallelCommandGroup(
    //             (new InstantCommand(() -> rotatingArm.setArmState(rotateState)).repeatedly()).until(() -> rotatingArm.atDesiredSetPoint()),
    //             (new InstantCommand(() -> wrist.setArmState(wristState)).repeatedly()).until(() -> wrist.atDesiredSetPoint())
    //         ),
    //         ((new InstantCommand(() -> cascadingArm.setArmState(cascadeState)).repeatedly()).until(() -> cascadingArm.atDesiredSetPoint()))
    //     );
    // }

    // public Command autoScoreLow(GameElementState gameElementState) {
    //     return new SequentialCommandGroup(
    //         updateLocalStates(false),
    //         setGameElementState(gameElementState),
    //         setPositionState(PositionState.LOW_SCORE),
    //         new WaitCommand(0.1),
    //         new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
    //         new WaitCommand(0.3),
    //         new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
    //         setPositionState(PositionState.TUCKED)
    //     );
    // }

    // public Command autoScoreMid(GameElementState gameElementState) {
    //     return new SequentialCommandGroup(
    //         updateLocalStates(false),
    //         setGameElementState(gameElementState),
    //         setPositionState(PositionState.MID_SCORE),
    //         new WaitCommand(0.1),
    //         new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
    //         new WaitCommand(0.3),
    //         new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
    //         setPositionState(PositionState.TUCKED)
    //     );
    // }

    // public Command autoScoreHigh(GameElementState gameElementState) {
    //     return new SequentialCommandGroup(
    //         updateLocalStates(false),
    //         setGameElementState(gameElementState),
    //         setPositionState(PositionState.HIG_SCORE),
    //         new WaitCommand(0.1),
    //         new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
    //         new WaitCommand(0.3),
    //         new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
    //         setPositionState(PositionState.TUCKED)
    //     );
    // }

    // public Command autoGroundIntake(GameElementState gameElementState) {
    //     return new SequentialCommandGroup(
    //         updateLocalStates(false),
    //         setGameElementState(gameElementState),
    //         setPositionState(PositionState.GROUND_INTAKING),
    //         new WaitCommand(0.1),
    //         new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.INTAKE)),
    //         new WaitCommand(0.3),
    //         new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
    //         setPositionState(PositionState.TUCKED)
    //     );
    // }
}
