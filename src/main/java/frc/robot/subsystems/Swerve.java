package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModule[] mSwerveMods;
    private Pigeon2 gyro;

    private TeleopState teleopState;
    private Rotation2d userGyroOffset; // gyro should not ever be zeroed during teleop, zero rotation is toward the positive x direction on the field -- facing the red alliance grid

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.drivetrainCANbusName);
        gyro.configFactoryDefault();
        zeroGyro();
        userGyroOffset = Rotation2d.fromDegrees(0);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        Timer.delay(0.1); // wow ok
        resetModulesToAbsolute(); // works but should preferably be threaded
        
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), new Pose2d()); //TODO: Fix

        teleopState = TeleopState.NORMAL;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) { // teleop manual driving
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        switch (teleopState) { 
            case NORMAL:
                swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation, 
                        getYaw().plus(userGyroOffset)
                    ) : new ChassisSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation)
                );
                break;
            case LOCKED:
                swerveModuleStates[0] = new SwerveModuleState(0.01, new Rotation2d(45)); // TODO: why need speed? lock wheels please
                swerveModuleStates[1] = new SwerveModuleState(0.01, new Rotation2d(-45));
                swerveModuleStates[2] = new SwerveModuleState(0.01, new Rotation2d(-45));
                swerveModuleStates[3] = new SwerveModuleState(0.01, new Rotation2d(45));
                break;
            case AIMBOT:
                DriverStation.reportError("AIMBOT", false); // If this is ever printed something bad has happened
                return;
                
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

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getPositions(), pose);
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void zeroGyroOffset() {
        userGyroOffset = Rotation2d.fromDegrees(0).minus(getYaw()); // makes the front of the robot for the driver the current front of the robot TODO: check if this works
    }

    public void setGyroOffset(Rotation2d offset) {
        userGyroOffset = offset;
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

    public void resetModulesToAbsolute(){ // tells the swerve modules to set the integrated encoders on the angle falcons based on the absolute cancoders
        for(SwerveModule mod : mSwerveMods)
            mod.resetToAbsolute();
    }

    public void updatePoseEstimator(Pose2d pose, double latency) { // updates the pose estimator from vision
        if (pose.getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation()) < Constants.VisionConstants.maximumOffset) // throws out bad vision data
            poseEstimator.addVisionMeasurement(pose, latency);
        else
            System.out.println("Bad vision data recieved");
    }

    public void setSwerveState(TeleopState newState) {
        teleopState = newState;
    }

    public static enum TeleopState {
        NORMAL,
        LOCKED, // for defense, makes an x formation
        AIMBOT
    }
}

