package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.WheeledIntakeConstants;
import frc.robot.Constants.WheeledIntakeConstants.WheeledIntakeState;

public class WheeledIntake {
    private CANSparkMax m1;
    private CANSparkMax m2;
    private WheeledIntakeState wheeledIntakeState;

    public WheeledIntake() {
        m1 = new CANSparkMax(WheeledIntakeConstants.wheeledIntake1Id, MotorType.kBrushless);
        m2 = new CANSparkMax(WheeledIntakeConstants.wheeledIntake2Id, MotorType.kBrushless);
        
        configMotors();

        wheeledIntakeState = WheeledIntakeState.NEUTRAL;
    }

    public void configMotors() {
        m1.restoreFactoryDefaults();
        m1.setSmartCurrentLimit(20, 10);
        m1.setIdleMode(IdleMode.kCoast);
        m1.setInverted(false);
        m1.burnFlash();

        m2.restoreFactoryDefaults();
        m2.setSmartCurrentLimit(20, 10);
        m2.setIdleMode(IdleMode.kCoast);
        m2.setInverted(true);
        m2.follow(m1);
        m2.burnFlash();
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