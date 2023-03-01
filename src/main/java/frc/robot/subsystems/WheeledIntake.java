package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.WheeledIntakeConstants;

public class WheeledIntake {
    private CANSparkMax m1;
    private CANSparkMax m2;

    public WheeledIntake() {
        m1 = new CANSparkMax(WheeledIntakeConstants.wheeledIntake1id, MotorType.kBrushless);
        m2 = new CANSparkMax(WheeledIntakeConstants.wheeledIntake2id, MotorType.kBrushless);
        m2.follow(m1);
        m2.setInverted(true);
    }

    public void intake(){
        m1.set(WheeledIntakeConstants.wheeledIntakeSpeed);
    }

    public void outtake(){
        m1.set(WheeledIntakeConstants.wheeledOuttakeSpeed);
    }
}
