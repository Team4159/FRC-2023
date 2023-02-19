package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CascadingArmConstants;

public class CascadingArm extends SubsystemBase {
    private TalonFX armTalon;
    private ArmState armState;

    public CascadingArm() {
        armTalon = new TalonFX(CascadingArmConstants.cascadingArmId);
        configMotor();

        armState = ArmState.OFF;
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
        switch (armState) {
            case INTAKING:
                setArmPosition(CascadingArmConstants.intakingSetpoint);
                break;
            case SCORING1:
                setArmPosition(CascadingArmConstants.scoringOneSetpoint);
                break;
            case SCORING2:
                setArmPosition(CascadingArmConstants.scoringTwoSetpoint);
                break;
            case SCORING3:
                setArmPosition(CascadingArmConstants.scoringThreeSetpoint);
                break;
            case TUCKED:
                setArmPosition(CascadingArmConstants.tuckedSetpoint);
                break;
            case OFF:
                armTalon.set(ControlMode.PercentOutput, 0);
                break;
        }
    }

    public double getEncoderPosition() {
        return armTalon.getSelectedSensorPosition();
    }

    public void setArmState(ArmState armState) {
        this.armState = armState;
    }

    public void setArmPosition(double position) {
        //speed = MathUtil.clamp(speed, CascadingArmConstants.lowSpeed, CascadingArmConstants.highSpeed);
        armTalon.set(ControlMode.Position, position);
    }

    public static enum ArmState {
        INTAKING,
        SCORING1,
        SCORING2,
        SCORING3,
        TUCKED,
        OFF
    }

    public boolean isAtSetpoint(double setpoint) {
        return getEncoderPosition() == setpoint;
    }
    
    public boolean isAtSetpoint(double setpoint, double tolerance) {
        double pos = getEncoderPosition();
        return setpoint - tolerance <= pos && setpoint + tolerance >= pos;
    }
}