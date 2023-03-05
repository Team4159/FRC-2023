package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CascadingArmConstants.CascadeState;
import frc.robot.Constants.RotatingArmConstants.RotateState;
import frc.robot.Constants.WristConstants.WristState;
import frc.robot.subsystems.LED.LEDState;

public class StateController extends SubsystemBase {
    private Swerve swerve;
    private LED led;
    private RotatingArm rotatingArm;
    private CascadingArm cascadingArm;
    private Wrist writst;
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
        swerve = RobotContainer.s_Swerve;
        led = RobotContainer.led;
        rotatingArm = RobotContainer.rotatingArm;
        cascadingArm = RobotContainer.cascadingArm;
        writst = RobotContainer.wrist;
        wheeledIntake = RobotContainer.wheeledIntake;


        gameElementState = GameElementState.CUBE;
        positionState = PositionState.TUCKED;
    }

    public void setGameElementState(GameElementState gameElementState) {
        this.gameElementState = gameElementState;
        updateLocalStates();
    }

    public void setPositionState(PositionState positionState) {
        this.positionState = positionState;
        updateLocalStates();
    }

    public void updateLocalStates() {
        switch (gameElementState) {
            case CONE:
                switch (positionState) {
                    case TUCKED:
                        led.setState(LEDState.YELLOW);
                        rotatingArm.setArmState(RotateState.TUCKED);
                        cascadingArm.setArmState(CascadeState.TUCKED);
                        writst.setArmState(WristState.TUCKED);
                        break;
                    case GROUND_INTAKING:
                        led.setState(LEDState.YELLOW);
                        
                        break;
                    case SINGLE_SUBSTATION:
                        led.setState(LEDState.YELLOW);

                        break;
                    case DOUBLE_SUBSTATION:
                        led.setState(LEDState.YELLOW);

                        break;
                    case LOW_SCORE:
                        led.setState(LEDState.RAINBOW);
                        
                        break;
                    case MID_SCORE:
                        led.setState(LEDState.RAINBOW);

                        break;
                    case HIG_SCORE:
                        led.setState(LEDState.RAINBOW);

                        break;
                }
                break;
            case CUBE:
                switch (positionState) {
                    case TUCKED:
                        led.setState(LEDState.PURPLE);

                        break;
                    case GROUND_INTAKING:
                        led.setState(LEDState.PURPLE);

                        break;
                    case SINGLE_SUBSTATION:
                        led.setState(LEDState.PURPLE);

                        break;
                    case DOUBLE_SUBSTATION:
                        led.setState(LEDState.PURPLE);

                        break;
                    case LOW_SCORE:
                        led.setState(LEDState.RAINBOW);

                        break;
                    case MID_SCORE:
                        led.setState(LEDState.RAINBOW);

                        break;
                    case HIG_SCORE:
                        led.setState(LEDState.RAINBOW);

                        break;
                }
                break;
        }
    }
}
