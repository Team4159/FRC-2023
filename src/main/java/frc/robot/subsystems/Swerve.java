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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private TeleopState teleopState;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.drivetrainCANbusName);
        gyro.configFactoryDefault();
        zeroGyro();
        
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
                break;
            case LOCKED:
                System.out.println("LOCKED"); // TODO: once tested, this print can be removed
                swerveModuleStates[0] = new SwerveModuleState(0, new Rotation2d(45)); // TODO: neeeeed to check for correct angle
                swerveModuleStates[1] = new SwerveModuleState(0, new Rotation2d(135)); // TODO
                swerveModuleStates[2] = new SwerveModuleState(0, new Rotation2d(135)); // TODO
                swerveModuleStates[3] = new SwerveModuleState(0, new Rotation2d(45)); // TODO
                break;
            case AIMBOT:
                DriverStation.reportError("AIMBOT", false); // If this is ever printed something bad has happened
                break;
                
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

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods)
            mod.resetToAbsolute();
    }

    public void updatePoseEstimator(Pose2d pose, double latency) {
        poseEstimator.addVisionMeasurement(pose, latency);
    } // it already checks to see if the measurement is too far off

    public void setSwerveState(TeleopState newState) {
        teleopState = newState;
    }

    public Pose2d getTargetPose(Pose2d initialPose) {

        String region = lookupRegion(initialPose);
        if (region.equals("M")) return null;

        Rotation2d rot = Rotation2d.fromDegrees((region.charAt(0) == 'B') ? 180 : 0); // point left is scoring in blue and right if scoring in red

        double x = (region.charAt(0) == 'B') ?  1.98 : 14.56; // trust the numbers, this one has the robot bumper sit an inch from the little seperators on the grid


        switch (region.charAt(1)) {
            case '9':
                return new Pose2d(x, 4.99, rot); // trust the numbers or yell at me (Cole) if you don't
            case '8':
                return new Pose2d(x, 4.43, rot);
            case '7':
                return new Pose2d(x, 3.87, rot);
            case '6':
                return new Pose2d(x, 3.31, rot);
            case '5':
                return new Pose2d(x, 2.75, rot);
            case '4':
                return new Pose2d(x, 2.19, rot);
            case '3':
                return new Pose2d(x, 1.63, rot);
            case '2':
                return new Pose2d(x, 1.07, rot);
            case '1':
                return new Pose2d(x, 0.51, rot); // programmed at 2am
        }
        
        
        return null; // if it can't find a pose, return null and cry
    }

    private String lookupRegion(Pose2d pose) {

        double x = pose.getX();
        double y = pose.getY();

        String ans;

        if (x <= 2.92) // all numbers are in meters because pose is in meters
            ans = "B";
        else if (x >= 13.62) // 16.54 (full width) - 2.92 (distance to charging station tip)
            ans = "R";
        else
            return "M";

        if (y >= 4.71 && y <= 5.50)
            return ans + 9;
        else if (y >= 4.15)
            return ans + 8;
        else if (y >= 3.59)
            return ans + 7;
        else if (y >= 3.03)
            return ans + 6;
        else if (y >= 2.47)
            return ans + 5;
        else if (y >= 1.91)
            return ans + 4;
        else if (y >= 1.35)
            return ans + 3;
        else if (y >= 0.79)
            return ans + 2;
        else
            return ans + 1;
    }

    /* NO DELETE >:( This took like four hours
     * 
     * 
     * Full width of coordinate system with origin at april tag origin: 651.22in = 16.54m
     * 
     * 
     * Region names:
     * 
     * Cone     B9       R9     4.99
     * Cube     B8       R8     4.43
     * Cone     B7       R7     3.87
     * Cone     B6       R6     3.31
     * Cube     B5   M   R5     2.75
     * Cone     B4       R4     2.19
     * Cone     B3       R3     1.63
     * Cube     B2       R2     1.07
     * Cone     B1       R1     0.51
     */

    public static enum TeleopState {
        NORMAL,
        LOCKED, // for defense, makes an x formation
        AIMBOT
    }
}