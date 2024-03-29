package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WheeledIntakeConstants;
import frc.robot.Constants.WheeledIntakeConstants.WheeledIntakeState;

public class WheeledIntake extends SubsystemBase{
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
        m1.setSmartCurrentLimit(20);
        m1.setIdleMode(IdleMode.kCoast);
        m1.setInverted(false);

        m2.restoreFactoryDefaults();
        m2.setSmartCurrentLimit(20);
        m2.setIdleMode(IdleMode.kCoast);
        // m2.follow(m1, true);
    }

    @Override
    public void periodic(){
        setWheeledIntake(wheeledIntakeState.set);
    }

    public void setWheeledIntake(double speed) {
        m1.set(speed);
        m2.set(-speed);
    }

    public void setWheeledIntakeState(WheeledIntakeState wheeledIntakeState){
        this.wheeledIntakeState = wheeledIntakeState;
    }

    public String getWheeledIntakeState() {
        if (this.wheeledIntakeState == WheeledIntakeState.INTAKE_CONE) {
            return "Intake cone";
        } else if (this.wheeledIntakeState == WheeledIntakeState.INTAKE_CUBE) {
            return "Intake cube";
        } else if (this.wheeledIntakeState == WheeledIntakeState.OUTTAKE_CUBE) {
            return "Outtake cube";
        } else if (this.wheeledIntakeState == WheeledIntakeState.OUTTAKE_CONE) {
            return "Outtake cone";
        } else if (this.wheeledIntakeState == WheeledIntakeState.NEUTRAL) {
            return "Neutral";
        } else {
            return "Off"; 
        }
    }
}