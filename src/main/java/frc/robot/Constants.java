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
import edu.wpi.first.wpilibj.DriverStation;
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
        public static final double maxAngularVelocity = 3.0;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(92.0215);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(283.2715);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(289.5117);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(199.8633);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final PathConstraints kPathConstraints = new PathConstraints(3, 3);
    
        public static final double kPXController = 15;
        public static final double kPYController = 15;
        public static final double kPThetaController = 10;

        public static final double maxAutobalanceSpeed = 4; // m/s for the fast ram phase
        public static final double maxAutobalanceUphillSpeed = 0.8; // m/s for the slowly going up phase
        public static final double maxAutobalanceReverseSpeed = 0.25; // m/s for the slow reverse phase
        public static final double autobalanceDebounceTime = 0.1; // seconds

        public static final double autoRotateTolerance = 10; // degrees
        public static final double autoRotateDebounceTime = 0.2; // seconds
    }

    public static final class RotatingArmConstants {
        public static enum RotateState {
            INITIAL(0),
            TUCKED_CUBE(10000), // 5000
            GROUND_INTAKE_CUBE(30000), // 19716
            DOUBLE_INTAKE_CUBE(87000), // 74890
            LOW_CUBE(0), // 0
            SEMI_TUCK(21600), // 21600
            MID_CUBE(41757), // 21600
            HIGH_CUBE(97847), // 21600
            TUCKED_CONE(10000), // 0
            GROUND_INTAKE_CONE(28000), // 14436
            DOUBLE_INTAKE_CONE(87000), // 85000
            LOW_CONE(0), // 30000
            MID_CONE(71000), // 76000
            MID_CONE_FORCE(70000), // 73000
            HIGH_CONE(97000), //93000
            HIGH_CONE_FORCE(95000), // 90000
            OFF(-1);

            public final double setpoint;
            RotateState(double setpoint) {
                if (setpoint > absoluteMaxSetpoint) {
                    this.setpoint = absoluteMaxSetpoint;
                    DriverStation.reportError("Trying to set rotation above absolute fields", false);
                    return;
                }
                this.setpoint = setpoint;
            }
        }

        public static final int rotatingArmID1 = 1; 
        public static final int rotatingArmID2 = 2;

        public static final double kP = 0.02;
        public static final double kI = 0;
        public static final double kD = 0.005;
        public static final double kF = 0;

        public static final double absoluteMaxSetpoint = 120000;
        
        public static final double setpointTolerance = 2000;

        public static final int rotateContinuousCurrentLimit = 35;
        public static final int rotatePeakCurrentLimit = 45;
        public static final double rotatePeakCurrentDuration = 0.1;
        public static final boolean rotateEnableCurrentLimit = true;
        
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.2;
        
        public static final boolean rotateMotorInvert = false;
        public static final NeutralMode rotateNeutralMode = NeutralMode.Brake;
    }
    
    public static final class CascadingArmConstants {
        public static enum CascadeState {
            INITIAL(0),
            TUCKED_CUBE(-1000), // 0
            GROUND_INTAKE_CUBE(-13000),
            DOUBLE_INTAKE_CUBE(0),
            LOW_CUBE(0), // 0
            MID_CUBE(0), // 0
            HIGH_CUBE(-25811), // 20811
            TUCKED_CONE(5000), // 0
            GROUND_INTAKE_CONE(-24688),
            DOUBLE_INTAKE_CONE(0),
            LOW_CONE(0), // 18400
            MID_CONE(-24000), /// 13500
            HIGH_CONE(-60000), //41000
            OFF(-1);

            public final double setpoint;
            CascadeState(double setpoint) {
                this.setpoint = setpoint;
            }
        }

        public static final int cascadingArmId = 3; 

        public static final double kP = 0.012; //0.004
        public static final double kI = 0;
        public static final double kD = 0.005;
        public static final double kF = 0;

        public static final double setpointTolerance = 2500;

        public static final int cascadeContinuousCurrentLimit = 30;
        public static final int cascadePeakCurrentLimit = 45;
        public static final double cascadePeakCurrentDuration = 0.1;
        public static final boolean cascadeEnableCurrentLimit = true;
        
        public static final double openLoopRamp = 0.05;
        public static final double closedLoopRamp = 0.5;

        public static final boolean cascadeMotorInvert = false;
        public static final NeutralMode cascadeNeutralMode = NeutralMode.Brake;
    }

    public static final class WristConstants {
        public static enum WristState {
            INITIAL(0),
            TUCKED_CUBE(10527), // 500
            GROUND_INTAKE_CUBE(20000), // 15000
            DOUBLE_INTAKE_CUBE(20000), // 18000
            LOW_CUBE(8386), // 14000
            MID_CUBE(10386), // 14000
            HIGH_CUBE(17000), // 14000
            TUCKED_CONE(8000), // 500
            GROUND_INTAKE_CONE(17000), // 15000
            DOUBLE_INTAKE_CONE(20000), // 18000
            LOW_CONE(10385), // 12000
            MID_CONE(9000), // 10000
            HIGH_CONE(13000), //10000
            FORCE_SCORE(16549), // 18000
            OFF(-1);

            public final double setpoint;
            WristState(double setpoint) {
                this.setpoint = setpoint;
            }
        }

        public static final int wristId = 4; 

        public static final double kP = 0.032;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static final double setpointTolerance = 200;

        public static final int wristContinuousCurrentLimit = 30;
        public static final int wristPeakCurrentLimit = 45;
        public static final double wristPeakCurrentDuration = 0.1;
        public static final boolean wristEnableCurrentLimit = true;
        
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final boolean wristMotorInvert = true;
        public static final NeutralMode wristNeutralMode = NeutralMode.Brake;
    }
    
    public static final class WheeledIntakeConstants {
        public static enum WheeledIntakeState{
            INTAKE_CUBE(0.2),
            INTAKE_CONE(0.6),
            OUTTAKE_CUBE(-0.25),
            OUTTAKE_CONE(-0.12),
            NEUTRAL(0.07),
            OFF(0);

            public final double set;
            WheeledIntakeState(double set) {
                this.set = set;
            }
        }
        
        public static final int wheeledIntake1Id = 6;
        public static final int wheeledIntake2Id = 17;
    }

    public static final class JoystickConstants {

        public static final class PrimaryDrive {
            public static final int drivePort = 0;

            public static final int zeroGyro = 1;
            //public static final int lockedMode = 2;
            public static final int toggleRotateLock = 2;
            public static final int rotateLockClockwise = 4;
            public static final int rotateLockCounterclockwise = 3;

            public static final int lockRotateCone = toggleRotateLock; // intentionally the same number as toggle rotate lock
        }

        public static final class PrimaryLeft {
            public static final int leftPort = 1;

            public static final int aimbot = 1;
            public static final int forceAcceptVision = 2;
        }

        public static final class Secondary {
            public static final int secondaryPort = 2;

            public static final int outtake = 1;
            public static final int intake = 2;

            public static final int cube = 3;
            public static final int cone = 4;

            public static final int high = 11;
            public static final int mid = 12;
            public static final int low = 13;

            public static final int groundIntake = 7;
            public static final int singleIntake = 6;
            public static final int doubleIntake = 5;

            public static final int forceScore = 15;
            
            // public static final int wristCascadeTuck = 8;
            // public static final int rotateTuck = 9;
            public static final int tucked = 8;
            public static final int leftTucked = 14;
            
            public static final int blinkLEDButton = 10; // TODO: PLACEHOLDER
        }
    }

    public static final class VisionConstants {
        public static final double fieldWidth = 16.54;
        public static final double fieldHeight= 8;
        public static final double maximumOffset = 1; // Meters
    }

    public static final class Fun {
        public static final int ledPort = 9;
        public static final int ledLength = 229;
        public static final double ledReductionFactor = 0.9;

        public static final double ledBlinkDelay = 0.5d;
        public static final double blinkLEDDelayDecrease = 1.5;
        public static final int blinkLEDDelayThreshold = 5;
    }
}
