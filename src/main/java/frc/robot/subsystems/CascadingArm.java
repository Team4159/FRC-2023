package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CascadingArmConstants;

public class CascadingArm extends SubsystemBase {
    private TalonFX armTalon;
    private CascadeState cascadeState;

    public CascadingArm() {
        armTalon = new TalonFX(CascadingArmConstants.cascadingArmId);
        configMotor();

        cascadeState = CascadeState.OFF;
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
        System.out.println("cascade: " + getEncoderPosition());
        switch (cascadeState) {
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

    public void setArmState(CascadeState cascadeState) {
        this.cascadeState = cascadeState;
    }

    public void setArmPosition(double position) {
        //speed = MathUtil.clamp(speed, CascadingArmConstants.lowSpeed, CascadingArmConstants.highSpeed);
        armTalon.set(ControlMode.Position, position);
    }

    public static enum CascadeState {
        INTAKING,
        SCORING1,
        SCORING2,
        SCORING3,
        TUCKED,
        OFF
    }

    public boolean atDesiredSetPoint() {
        double setpoint = 0;
        switch (cascadeState) {
            case SCORING1:
                setpoint = CascadingArmConstants.scoringOneSetpoint;
                break;
            case SCORING2:
                setpoint = CascadingArmConstants.scoringTwoSetpoint;
                break;
            case SCORING3:
                setpoint = CascadingArmConstants.scoringThreeSetpoint;
                break;
            case INTAKING:
                setpoint = CascadingArmConstants.intakingSetpoint;
                break;
            case TUCKED:
                setpoint = CascadingArmConstants.tuckedSetpoint;
                break;
            case OFF:
                return true;
        }
        return isAtSetpoint(setpoint, CascadingArmConstants.setpointTolerance);
    }

    public boolean isAtSetpoint(double setpoint) {
        return getEncoderPosition() == setpoint;
    }
    
    public boolean isAtSetpoint(double setpoint, double tolerance) {
        double pos = getEncoderPosition();
        return Math.abs(setpoint-pos) < tolerance;
    }
}