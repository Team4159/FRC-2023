package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RotatingArmConstants;

public class RotatingArm extends SubsystemBase {
    private TalonFX armTalon1;
    private TalonFX armTalon2;
    private RotateState rotateState;

    public RotatingArm() {
        armTalon1 = new TalonFX(RotatingArmConstants.rotatingArmID1);
        armTalon2 = new TalonFX(RotatingArmConstants.rotatingArmID2);
        configMotors();

        rotateState = RotateState.OFF;
    }

    public void configMotors() {
        armTalon1.configFactoryDefault();
        armTalon1.configAllSettings(Robot.ctreConfigs.rotateFXConfig);
        armTalon1.setInverted(RotatingArmConstants.rotateMotorInvert);
        armTalon1.setNeutralMode(RotatingArmConstants.rotateNeutralMode);
        armTalon1.setSelectedSensorPosition(0); // resets the arm talon encoder to 0

        armTalon2.configFactoryDefault();
        armTalon2.configAllSettings(Robot.ctreConfigs.rotateFXConfig);
        armTalon2.setInverted(RotatingArmConstants.rotateMotorInvert);
        armTalon2.setNeutralMode(RotatingArmConstants.rotateNeutralMode);
        armTalon2.setSelectedSensorPosition(0); // resets the arm talon encoder to 0 
        armTalon2.follow(armTalon1);
    }

    @Override
    public void periodic() {
        System.out.println("rotate: " + getEncoderPosition());
        switch (rotateState) {
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

    public void setArmState(RotateState rotateState) {
        this.rotateState = rotateState;
    }

    public void setArmPosition(double position) {
        armTalon1.set(ControlMode.Position, position);
    }

    public static enum RotateState {
        LOW,
        MID,
        HIGH,
        INTAKING,
        TUCKED,
        OFF
    }

    public boolean atDesiredSetPoint() {
        double setpoint = 0;
        switch (rotateState) {
            case LOW:
                setpoint = RotatingArmConstants.lowSetpoint;
                break;
            case MID:
                setpoint = RotatingArmConstants.midSetpoint;
                break;
            case HIGH:
                setpoint = RotatingArmConstants.highSetpoint;
                break;
            case INTAKING:
                setpoint = RotatingArmConstants.intakingSetpoint;
                break;
            case TUCKED:
                setpoint = RotatingArmConstants.tuckedSetpoint;
                break;
            case OFF:
                return true;
        }
        return isAtSetpoint(setpoint, RotatingArmConstants.setpointTolerance);
    }

    public boolean isAtSetpoint(double setpoint) {
        return getEncoderPosition() == setpoint;
    }
    
    public boolean isAtSetpoint(double setpoint, double tolerance) {
        double pos = getEncoderPosition();
        return Math.abs(setpoint-pos) < tolerance;
    }
}
