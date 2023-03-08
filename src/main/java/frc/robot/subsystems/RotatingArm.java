package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RotatingArmConstants;
import frc.robot.Constants.RotatingArmConstants.RotateState;
import frc.robot.Robot;

public class RotatingArm extends SubsystemBase {
    private TalonFX armTalon1, armTalon2;
    private RotateState rotateState;

    public RotatingArm() {
        armTalon1 = new TalonFX(RotatingArmConstants.rotatingArmID1);
        armTalon2 = new TalonFX(RotatingArmConstants.rotatingArmID2);
        configMotors();

        rotateState = RotateState.INITIAL;
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
        if (rotateState.equals(RotateState.OFF)) armTalon1.set(ControlMode.PercentOutput, 0);
        setArmPosition(rotateState.setpoint);
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
    
    public boolean atDesiredSetPoint() {
        if (rotateState.equals(RotateState.OFF)) return true;
        return isAtSetpoint(rotateState.setpoint, RotatingArmConstants.setpointTolerance);
    }

    public boolean isAtSetpoint(double setpoint) {
        return getEncoderPosition() == setpoint;
    }
    
    public boolean isAtSetpoint(double setpoint, double tolerance) {
        return Math.abs(setpoint-getEncoderPosition()) < tolerance;
    }
}
