package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.WheeledIntakeConstants;

public class WheeledIntake {
    private CANSparkMax m1;
    private CANSparkMax m2;
    private WheeledIntakeState wheeledIntakeState;

    public WheeledIntake() {
        m1 = new CANSparkMax(WheeledIntakeConstants.wheeledIntake1id, MotorType.kBrushless);
        m2 = new CANSparkMax(WheeledIntakeConstants.wheeledIntake2id, MotorType.kBrushless);
        m2.follow(m1);
        m2.setInverted(true);
        wheeledIntakeState = WheeledIntakeState.OFF;
    }

    public void configMotors(){}

    public void periodic(){
        switch (wheeledIntakeState){
            case INTAKE:
            m1.set(WheeledIntakeConstants.wheeledIntakeSpeed);
            break;
            case OUTTAKE:
            m1.set(WheeledIntakeConstants.wheeledOuttakeSpeed);
            break;
            case OFF:
            m1.set(0);
            break;
        }
    }

    public static enum WheeledIntakeState{
        INTAKE,
        OUTTAKE,
        OFF
    }
}