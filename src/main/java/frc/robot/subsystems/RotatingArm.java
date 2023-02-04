package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RotatingArm extends SubsystemBase {
    private TalonFX armTalon;
    private PIDController pid;
    private ArmState armState;

    public RotatingArm() {
        armTalon = new TalonFX(Constants.RotatingArmConstants.rotatingArmID);
        pid = new PIDController(Constants.RotatingArmConstants.kP,
                                Constants.RotatingArmConstants.kI,
                                Constants.RotatingArmConstants.kD);
        armState = ArmState.OFF;
    }

    @Override
    public void periodic() {
        switch (armState) {
            case LOW:
                setArmSpeed(runArmPID(armTalon.getSelectedSensorPosition(), Constants.RotatingArmConstants.lowSetpoint));
                break;
            case MID:
                setArmSpeed(runArmPID(armTalon.getSelectedSensorPosition(), Constants.RotatingArmConstants.midSetpoint));
                break;
            case HIGH:
                setArmSpeed(runArmPID(armTalon.getSelectedSensorPosition(), Constants.RotatingArmConstants.highSetpoint));
                break;
            case OFF:
                setArmSpeed(runArmPID(armTalon.getSelectedSensorPosition(), Constants.RotatingArmConstants.offSetpoint));
                break;
        }
    }

    public void setArmState(ArmState armState) {
        this.armState = armState;
    }

    public void setArmSpeed(double speed) {
        speed = MathUtil.clamp(speed, -0.5, 0.5);
        armTalon.set(ControlMode.Velocity, speed);
    }

    public double runArmPID(double currentPos, double setPoint) {
        return pid.calculate(currentPos, setPoint);
    }

    public static enum ArmState {
        LOW,
        MID,
        HIGH,
        OFF
    }
}
