package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.WheeledIntakeConstants;
import frc.robot.Constants.WheeledIntakeConstants.WheeledIntakeState;

public class WheeledIntake {
    private CANSparkMax m1;
    private CANSparkMax m2;
    private WheeledIntakeState wheeledIntakeState;

    public WheeledIntake() {
        m1 = new CANSparkMax(WheeledIntakeConstants.wheeledIntake1id, MotorType.kBrushless);
        m2 = new CANSparkMax(WheeledIntakeConstants.wheeledIntake2id, MotorType.kBrushless);
        m2.setInverted(true);
        m2.follow(m1);
        wheeledIntakeState = WheeledIntakeState.NEUTRAL;
    }

    public void periodic(){
        setWheeledIntake(wheeledIntakeState.set);
    }

    public void setWheeledIntake(double speed) {
        m1.set(speed);
    }

    public void setWheeledIntakeState(WheeledIntakeState wheeledIntakeState){
        this.wheeledIntakeState = wheeledIntakeState;
    }

}