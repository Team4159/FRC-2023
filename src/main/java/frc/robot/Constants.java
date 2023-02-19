package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final String drivetrainCANbusName = "Drivetrain";

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)); // standard deviations for x, y, and rotation for gyro/encoder measurements. This is how much the numbers are trusted -- bigger number = less trust
        public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)); // standard deviations for x, y, and rotation for vision

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.75); 
        public static final double wheelBase = Units.inchesToMeters(23.75); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.07;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0; // no touching

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.2424 / 12); // TODO: get constants again when robot heavier
        public static final double driveKV = (0.7146 / 12);
        public static final double driveKA = (0.0773 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(92.9004);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(283.1836);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(289.9512);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(200.8301);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final PathConstraints kPathConstraints = new PathConstraints(4, 3);
    
        public static final double kPXController = 15;
        public static final double kPYController = 15;
        public static final double kPThetaController = 10;
    }

    public static final class RotatingArmConstants { // TODO: tune
        public static final int rotatingArmID1 = 0; 
        public static final int rotatingArmID2 = 0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static final int rotateContinuousCurrentLimit = 35;
        public static final int rotatePeakCurrentLimit = 45;
        public static final double rotatePeakCurrentDuration = 0.1;
        public static final boolean rotateEnableCurrentLimit = true;
        
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        
        public static final boolean rotateMotorInvert = false;
        public static final NeutralMode rotateNeutralMode = NeutralMode.Brake;

        public static final double intakingSetpoint = 0;
        public static final double lowSetpoint = 0;
        public static final double midSetpoint = 0;
        public static final double highSetpoint = 0;
        public static final double tuckedSetpoint = 0;

        public static final double lowSpeed = -1; // maximum speed rotating down
        public static final double highSpeed = 1; // maximum speed rotating up
    }
    
    public static final class CascadingArmConstants {
        public static final int cascadingArmId = 0; 

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static final int cascadeContinuousCurrentLimit = 30;
        public static final int cascadePeakCurrentLimit = 45;
        public static final double cascadePeakCurrentDuration = 0.1;
        public static final boolean cascadeEnableCurrentLimit = true;
        
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final boolean cascadeMotorInvert = false;
        public static final NeutralMode cascadeNeutralMode = NeutralMode.Brake;

        public static final double intakingSetpoint = 0;
        public static final double scoringOneSetpoint = 0;
        public static final double scoringTwoSetpoint = 0;
        public static final double scoringThreeSetpoint = 0;
        public static final double tuckedSetpoint = 0;

        public static final double lowSpeed = -1; // maximum speed retracting
        public static final double highSpeed = 1; // maximum speed extending
    }

    public static final class JoystickConstants {

        public static final class PrimaryDrive {
            public static final int drivePort = 0;

            public static final int zeroGyro = 1;
            public static final int lockedMode = 2;
        }

        public static final class PrimaryLeft {
            public static final int leftPort = 1;

            public static final int aimbot = 1;
            public static final int forceAcceptVision = 2;
        }

        public static final class Secondary {
            public static final int secondaryPort = 2;

            public static final int togglePincerArm = 1;

        }

    }

    public static final class VisionConstants {
        public static final double fieldWidth = 16.54;
        public static final double fieldHeight= 8;
        public static final double maximumOffset = 1; // Meters
    }
}
