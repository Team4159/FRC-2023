package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.WristState;
import frc.robot.Robot;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {

    private TalonFX wristTalon;
    private WristState wristState;

    private Debouncer setpointDebouncer;

    public Wrist() {
        wristTalon = new TalonFX(WristConstants.wristId);
        configMotor();

        setpointDebouncer = new Debouncer(0.2, DebounceType.kRising);

        wristState = WristState.INITIAL;
    }

    private void configMotor() {
        wristTalon.configFactoryDefault();
        wristTalon.configAllSettings(Robot.ctreConfigs.wristFXConfig);
        wristTalon.setInverted(WristConstants.wristMotorInvert);
        wristTalon.setNeutralMode(WristConstants.wristNeutralMode);
        wristTalon.setSelectedSensorPosition(0); // resets the arm talon encoder to 0 
    }
    
    @Override
    public void periodic() {
        if (wristState.equals(WristState.OFF)) wristTalon.set(ControlMode.PercentOutput, 0);
        setArmPosition(wristState.setpoint);
    }

    public double getEncoderPosition() {
        return wristTalon.getSelectedSensorPosition();
    }

    public void setArmState(WristState wristState) {
        this.wristState = wristState;
    }

    public void setArmPosition(double position) {
        wristTalon.set(ControlMode.Position, position);
    }

    public boolean atDesiredSetPoint() {
        // return true;
        if (wristState.equals(WristState.OFF)) return true;
        return isAtSetpoint(wristState.setpoint, WristConstants.setpointTolerance);
    }

    public boolean atDebouncedSetPoint() {
        return setpointDebouncer.calculate(atDesiredSetPoint());
    }

    public boolean isAtSetpoint(double setpoint) {
        return getEncoderPosition() == setpoint;
    }
    
    public boolean isAtSetpoint(double setpoint, double tolerance) {
        return Math.abs(setpoint-getEncoderPosition()) < tolerance;
    }
}
