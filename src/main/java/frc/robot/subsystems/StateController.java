package frc.robot.subsystems;

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
    private boolean forceScore;

    private int phase;

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


        gameElementState = GameElementState.CUBE;
        positionState = PositionState.TUCKED;
        forceScore = false;
        phase = 0;
    }

    @Override
    public void periodic() {
        System.out.println("Position: " + positionState);
        System.out.println("Element: " + gameElementState);
    }

    public void setGameElementState(GameElementState gameElementState) {
        this.gameElementState = gameElementState;
        cascadingArm.resetIntegrator();
        rotatingArm.resetIntegrator();
        updateLocalStates();
    }

    public boolean isGameElementCone() {
        return gameElementState == GameElementState.CONE;
    }

    public void setPositionState(PositionState positionState) {
        if (this.positionState == positionState) {
            phase++;
        } else {
            phase = 0;
        }
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
                setLocalStates(CascadeState.TUCKED_CONE, (phase == 0) ? RotateState.MID_CONE : RotateState.TUCKED_CONE, WristState.TUCKED_CONE);
            } else if (positionState == PositionState.GROUND_INTAKING) {
                led.setState(LEDState.YELLOW);
                System.out.println("cone ground");
                setLocalStates((phase == 0) ? CascadeState.TUCKED_CONE : CascadeState.GROUND_INTAKE_CONE, RotateState.GROUND_INTAKE_CONE, WristState.GROUND_INTAKE_CONE);
            } else if (positionState == PositionState.SINGLE_SUBSTATION) {
                led.setState(LEDState.YELLOW);
                System.out.println("cone single");
                setLocalStates(null, null, null);
            } else if (positionState == PositionState.DOUBLE_SUBSTATION) {
                led.setState(LEDState.YELLOW);
                System.out.println("cone double");
                setLocalStates((phase == 0) ? CascadeState.TUCKED_CONE : CascadeState.DOUBLE_INTAKE_CONE, RotateState.DOUBLE_INTAKE_CONE, WristState.DOUBLE_INTAKE_CONE);
            } else if (positionState == PositionState.LOW_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cone low");
                setLocalStates((phase == 0) ? CascadeState.TUCKED_CONE : CascadeState.LOW_CONE, RotateState.LOW_CONE, WristState.LOW_CONE);
            } else if (positionState == PositionState.MID_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cone mid");
                setLocalStates((phase == 0) ? CascadeState.TUCKED_CONE : CascadeState.MID_CONE, (forceScore) ? RotateState.MID_CONE_FORCE : RotateState.MID_CONE, WristState.MID_CONE);
            } else if (positionState == PositionState.HIG_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cone high");
                setLocalStates((phase == 0) ? CascadeState.TUCKED_CONE : CascadeState.HIGH_CONE, (forceScore) ? RotateState.HIGH_CONE_FORCE : RotateState.HIGH_CONE, WristState.HIGH_CONE);
            }
        } else if (gameElementState == GameElementState.CUBE) {
            if (positionState == PositionState.TUCKED) {
                led.setState(LEDState.PURPLE);
                System.out.println("cube tuck");
                setLocalStates(CascadeState.TUCKED_CUBE, (phase == 0) ? RotateState.LOW_CUBE : RotateState.TUCKED_CUBE, WristState.TUCKED_CUBE);
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
                setLocalStates((phase == 0) ? CascadeState.TUCKED_CUBE : CascadeState.DOUBLE_INTAKE_CUBE, RotateState.DOUBLE_INTAKE_CUBE, WristState.DOUBLE_INTAKE_CUBE);
            } else if (positionState == PositionState.LOW_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cube low");
                setLocalStates(CascadeState.LOW_CUBE, RotateState.LOW_CUBE, WristState.LOW_CUBE);
            } else if (positionState == PositionState.MID_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cube mid");
                setLocalStates((phase == 0) ? CascadeState.TUCKED_CUBE : CascadeState.MID_CUBE, RotateState.MID_CUBE, WristState.MID_CUBE);
            } else if (positionState == PositionState.HIG_SCORE) {
                led.setState(LEDState.RAINBOW);
                System.out.println("cube high");
                setLocalStates((phase == 0) ? CascadeState.TUCKED_CUBE : CascadeState.HIGH_CUBE, RotateState.HIGH_CUBE, WristState.HIGH_CUBE);
            }
        }
    }
}
