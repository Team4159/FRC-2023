package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public TalonFXConfiguration cascadeFXConfig;
    public TalonFXConfiguration rotateFXConfig;
    public TalonFXConfiguration wristFXConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        cascadeFXConfig = new TalonFXConfiguration();
        rotateFXConfig = new TalonFXConfiguration();
        wristFXConfig = new TalonFXConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.angleEnableCurrentLimit, 
            Constants.Swerve.angleContinuousCurrentLimit, 
            Constants.Swerve.anglePeakCurrentLimit, 
            Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.driveEnableCurrentLimit, 
            Constants.Swerve.driveContinuousCurrentLimit, 
            Constants.Swerve.drivePeakCurrentLimit, 
            Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Cascade Motor Configuration */
        SupplyCurrentLimitConfiguration cascadeSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.CascadingArmConstants.cascadeEnableCurrentLimit,
            Constants.CascadingArmConstants.cascadeContinuousCurrentLimit,
            Constants.CascadingArmConstants.cascadePeakCurrentLimit,
            Constants.CascadingArmConstants.cascadePeakCurrentDuration
        );

        cascadeFXConfig.slot0.kP = Constants.CascadingArmConstants.kP;
        cascadeFXConfig.slot0.kI = Constants.CascadingArmConstants.kI;
        cascadeFXConfig.slot0.kD = Constants.CascadingArmConstants.kD;
        cascadeFXConfig.slot0.kF = Constants.CascadingArmConstants.kF;
        cascadeFXConfig.supplyCurrLimit = cascadeSupplyLimit;
        cascadeFXConfig.openloopRamp = Constants.CascadingArmConstants.openLoopRamp;
        cascadeFXConfig.closedloopRamp = Constants.CascadingArmConstants.closedLoopRamp;
        
        /* Rotate Motor Configuration */
        SupplyCurrentLimitConfiguration rotateSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.RotatingArmConstants.rotateEnableCurrentLimit,
            Constants.RotatingArmConstants.rotateContinuousCurrentLimit,
            Constants.RotatingArmConstants.rotatePeakCurrentLimit,
            Constants.RotatingArmConstants.rotatePeakCurrentDuration
        );

        rotateFXConfig.slot0.kP = Constants.RotatingArmConstants.kP;
        rotateFXConfig.slot0.kI = Constants.RotatingArmConstants.kI;
        rotateFXConfig.slot0.kD = Constants.RotatingArmConstants.kD;
        rotateFXConfig.slot0.kF = Constants.RotatingArmConstants.kF;
        rotateFXConfig.supplyCurrLimit = rotateSupplyLimit;
        rotateFXConfig.openloopRamp = Constants.RotatingArmConstants.openLoopRamp;
        rotateFXConfig.closedloopRamp = Constants.RotatingArmConstants.closedLoopRamp;
        
        /* Wrist Motor Configuration */
        SupplyCurrentLimitConfiguration wristSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.WristConstants.wristEnableCurrentLimit,
            Constants.WristConstants.wristContinuousCurrentLimit,
            Constants.WristConstants.wristPeakCurrentLimit,
            Constants.WristConstants.wristPeakCurrentDuration
        );

        wristFXConfig.slot0.kP = Constants.WristConstants.kP;
        wristFXConfig.slot0.kI = Constants.WristConstants.kI;
        wristFXConfig.slot0.kD = Constants.WristConstants.kD;
        wristFXConfig.slot0.kF = Constants.WristConstants.kF;
        wristFXConfig.supplyCurrLimit = wristSupplyLimit;
        wristFXConfig.openloopRamp = Constants.WristConstants.openLoopRamp;
        wristFXConfig.closedloopRamp = Constants.WristConstants.closedLoopRamp;
    }
}