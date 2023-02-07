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
            case INTAKING:
                setArmSpeed(runArmPID(getEncoderPosition(), Constants.CascadingArmConstants.intakingSetpoint));
                break;
            case SCORING1:
                setArmSpeed(runArmPID(getEncoderPosition(), Constants.CascadingArmConstants.scoringOneSetpoint));
                break;
            case SCORING2:
                setArmSpeed(runArmPID(getEncoderPosition(), Constants.CascadingArmConstants.scoringTwoSetpoint));
                break;
            case SCORING3:
                setArmSpeed(runArmPID(getEncoderPosition(), Constants.CascadingArmConstants.scoringThreeSetpoint));
                break;
            case OFF:
                setArmSpeed(0);
                break;
        }
    }

    public double getEncoderPosition() {
        return armSpark.getEncoder().getPosition();
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
        SCORING3,
        OFF
    }
}