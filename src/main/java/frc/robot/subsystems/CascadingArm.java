package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CascadingArm extends SubsystemBase {
    private CANSparkMax armSpark;
    private PIDController pid;
    private ArmState armState;

    public CascadingArm() {
        armSpark = new CANSparkMax(Constants.CascadingArmConstants.cascadingArmId, MotorType.kBrushless);
        pid = new PIDController(Constants.CascadingArmConstants.kP,
                                Constants.CascadingArmConstants.kI,
                                Constants.CascadingArmConstants.kD);
        armState = ArmState.OFF;
    }

    @Override
    public void periodic() {
        switch (armState) {
            case LOW:
                setArmSpeed(runArmPID(armSpark.getSelectedSensorPosition(), Constants.CascadingArmConstants.lowSetpoint));
                break;
            case MID:
                setArmSpeed(runArmPID(armSpark.getSelectedSensorPosition(), Constants.CascadingArmConstants.midSetpoint));
                break;
            case HIGH:
                setArmSpeed(runArmPID(armSpark.getSelectedSensorPosition(), Constants.CascadingArmConstants.highSetpoint));
                break;
            case OFF:
                setArmSpeed(runArmPID(armSpark.getSelectedSensorPosition(), Constants.CascadingArmConstants.offSetpoint));
                break;
        }
    }

    public void setArmState(ArmState armState) {
        this.armState = armState;
    }

    public void setArmSpeed(double speed) {
        speed = MathUtil.clamp(speed, -0.5, 0.5);
        armSpark.set(speed);
    }

    public double runArmPID(double currentPos, double setPoint) {
        return pid.calculate(currentPos, setPoint);
    }


    public static enum ArmState {
        INTAKING,
        SCORING1,
        SCORING2,
        OFF
    }
}