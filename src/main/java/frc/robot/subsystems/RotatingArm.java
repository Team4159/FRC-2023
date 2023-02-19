package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RotatingArmConstants;;

public class RotatingArm extends SubsystemBase {
    private TalonFX armTalon1;
    private TalonFX armTalon2;
    private ArmState armState;

    public RotatingArm() {
        armTalon1 = new TalonFX(RotatingArmConstants.rotatingArmID1);
        armTalon2 = new TalonFX(RotatingArmConstants.rotatingArmID2);
        configMotors();

        armState = ArmState.OFF;
    }

    public void configMotors() {
        armTalon1.configFactoryDefault();
        armTalon1.configAllSettings(Robot.ctreConfigs.cascadeFXConfig);
        armTalon1.setInverted(RotatingArmConstants.rotateMotorInvert);
        armTalon1.setNeutralMode(RotatingArmConstants.rotateNeutralMode);
        armTalon1.setSelectedSensorPosition(0); // resets the arm talon encoder to 0

        armTalon2.configFactoryDefault();
        armTalon2.configAllSettings(Robot.ctreConfigs.cascadeFXConfig);
        armTalon2.setInverted(RotatingArmConstants.rotateMotorInvert);
        armTalon2.setNeutralMode(RotatingArmConstants.rotateNeutralMode);
        armTalon2.setSelectedSensorPosition(0); // resets the arm talon encoder to 0 
        armTalon2.follow(armTalon1);
    }

    @Override
    public void periodic() {
        switch (armState) {
            case LOW:
                setArmPosition(RotatingArmConstants.lowSetpoint);
                break;
            case MID:
                setArmPosition(RotatingArmConstants.midSetpoint);
                break;
            case HIGH:
                setArmPosition(RotatingArmConstants.highSetpoint);
                break;
            case INTAKING:
                setArmPosition(RotatingArmConstants.intakingSetpoint);
                break;
            case TUCKED:
                setArmPosition(RotatingArmConstants.tuckedSetpoint);
                break;
            case OFF:
                armTalon1.set(ControlMode.PercentOutput, 0);
                break;
        }
    }

    public double getEncoderPosition() {
        return armTalon1.getSelectedSensorPosition();
    }

    public void setArmState(ArmState armState) {
        this.armState = armState;
    }

    public void setArmPosition(double speed) {
        speed = MathUtil.clamp(speed, RotatingArmConstants.lowSpeed, RotatingArmConstants.highSpeed);
        armTalon1.set(ControlMode.Velocity, speed);
    }

    public static enum ArmState {
        LOW,
        MID,
        HIGH,
        INTAKING,
        TUCKED,
        OFF
    }
}
