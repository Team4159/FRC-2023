package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CascadingArmConstants;
import frc.robot.Constants.CascadingArmConstants.CascadeState;

public class CascadingArm extends SubsystemBase {
    private TalonFX armTalon;
    private CascadeState cascadeState;

    public CascadingArm() {
        armTalon = new TalonFX(CascadingArmConstants.cascadingArmId);
        configMotor();

        cascadeState = CascadeState.INITIAL;
    }

    public void configMotor() {
        armTalon.configFactoryDefault();
        armTalon.configAllSettings(Robot.ctreConfigs.cascadeFXConfig);
        armTalon.setInverted(CascadingArmConstants.cascadeMotorInvert);
        armTalon.setNeutralMode(CascadingArmConstants.cascadeNeutralMode);
        armTalon.setSelectedSensorPosition(0); // resets the arm talon encoder to 0 
    }

    @Override
    public void periodic() {
        if (cascadeState.equals(CascadeState.OFF)) armTalon.set(ControlMode.PercentOutput, 0);
        setArmPosition(cascadeState.setpoint);
    }

    public double getEncoderPosition() {
        return armTalon.getSelectedSensorPosition();
    }

    public void setArmState(CascadeState cascadeState) {
        this.cascadeState = cascadeState;
    }

    public void setArmPosition(double position) {
        armTalon.set(ControlMode.Position, position);
    }

    public boolean atDesiredSetPoint() {
        System.out.println(isAtSetpoint(cascadeState.setpoint, CascadingArmConstants.setpointTolerance));
        if (cascadeState.equals(CascadeState.OFF)) return true;
        return isAtSetpoint(cascadeState.setpoint, CascadingArmConstants.setpointTolerance);
    }

    public boolean isAtSetpoint(double setpoint) {
        return getEncoderPosition() == setpoint;
    }
    
    public boolean isAtSetpoint(double setpoint, double tolerance) {
        return Math.abs(setpoint-getEncoderPosition()) < tolerance;
    }
}