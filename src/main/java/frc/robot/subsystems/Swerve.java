package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private Pose2d targetAimPose;

    private TeleopState teleopState;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), /*__initialpose__*/ null);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        teleopState = TeleopState.NORMAL;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        switch (teleopState) { 
            case NORMAL:
                swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation, 
                        getYaw()
                    ) : new ChassisSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation)
                );
            case LOCKED:
                swerveModuleStates[0] = new SwerveModuleState(0, new Rotation2d(45)); // neeeeed to check
                swerveModuleStates[1] = new SwerveModuleState(0, new Rotation2d(45));
                swerveModuleStates[2] = new SwerveModuleState(0, new Rotation2d(45));
                swerveModuleStates[3] = new SwerveModuleState(0, new Rotation2d(45));
            case AIMBOT:
                Transform2d difference = targetAimPose.minus(getPose());
                swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        MathUtil.clamp(difference.getX(), -1, 1), 
                        MathUtil.clamp(difference.getY(), -1, 1), 
                        difference.getRotation().getDegrees()/180d,
                        getYaw()
                        )
                );
                
        } 

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods)
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for (SwerveModule mod : mSwerveMods)
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    /* End */

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[mSwerveMods.length];
        for (SwerveModule mod : mSwerveMods)
            states[mod.moduleNumber] = mod.getState();
        return states;
    }

    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for (SwerveModule mod : mSwerveMods)
            positions[mod.moduleNumber] = mod.getPosition();
        return positions;
    }

    public void setTargetPose(Pose2d pose) {
        targetAimPose = pose;
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getPositions(), pose);
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic(){
        poseEstimator.update(getYaw(), getPositions());  

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    public void updatePoseEstimator(Pose2d pose, double latency) {
        if (this.getPose().getTranslation().getDistance(pose.getTranslation()) < 1) 
            poseEstimator.addVisionMeasurement(pose, latency);
        else
            DriverStation.reportWarning("WARNING: Vision measurements are too far from poseEstimator!", false);
    }

    public static enum TeleopState {
        NORMAL,
        LOCKED, // for defense, makes an x formation
        AIMBOT
    }
}