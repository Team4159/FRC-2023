package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.FieldRegion;
import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Swerve extends SubsystemBase {

    private final SwerveModule[] mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    public Pigeon2 gyro;

    private TeleopState teleopState;
    private Rotation2d userGyroOffset; // gyro should not ever be zeroed during teleop, zero rotation is toward the positive x direction on the field -- facing the red alliance grid

    private boolean forceAcceptNextVision;

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics, 
        new Rotation2d(), 
        getPositions(), 
        new Pose2d(),
        Constants.Swerve.stateStdDevs,
        Constants.Swerve.visionMeasurementStdDevs
    );

    public static enum LockRotateState {
        RIGHT(0),
        UP(90),
        LEFT(180),
        DOWN(270),
        OFF(-1);

        public final double direction;
        LockRotateState(double direction) {
            this.direction = direction;
        }
    }

    private PIDController lockRotatePID; // TODO: test

    private LockRotateState lockRotateState;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.drivetrainCANbusName);
        gyro.configFactoryDefault();
        zeroGyro();
        userGyroOffset = Rotation2d.fromDegrees(0);
        poseEstimator.resetPosition(getYaw(), getPositions(), new Pose2d());

        Timer.delay(0.1); // wow ok
        resetModulesToAbsolute(); // works but should preferably be threaded

        lockRotatePID = new PIDController(AutoConstants.kPThetaController, 0, 0);
        lockRotatePID.enableContinuousInput(0, 360); //optimizes the rotation so it takes the shortest path :)

        teleopState = TeleopState.NORMAL;
        lockRotateState = LockRotateState.OFF;

        forceAcceptNextVision = false;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) { // teleop manual driving
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        switch (teleopState) { 
            case NORMAL:
                swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        lockRotateState == LockRotateState.OFF
                            ? rotation
                            : calculateLockRotate(), 
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

    private double calculateLockRotate() {
        return lockRotatePID.calculate(getPose().getRotation().getDegrees(), lockRotateState.direction);
    }

    public void disableLockRotate() {
        lockRotateState = LockRotateState.OFF;
    }

    public void lockRotateClosest() {
        double rot = getPose().getRotation().getDegrees();

        if (rot >= 315 || rot <= 45) {
            lockRotateState = LockRotateState.RIGHT;
        }
        else if (rot >= 45 && rot <= 135) {
            lockRotateState = LockRotateState.RIGHT;
        }
        else if (rot >= 135 && rot <= 225) {
            lockRotateState = LockRotateState.RIGHT;
        }
        else if (rot >= 225 && rot <= 315) {
            lockRotateState = LockRotateState.RIGHT;
        }
        else {
            System.out.println("Can't find closest locked rotation");
        }
    }

    public void lockRotateClockwise() {
        switch (lockRotateState) {
            case RIGHT:
                lockRotateState = LockRotateState.DOWN;
                break;
            case DOWN:
                lockRotateState = LockRotateState.LEFT;
                break;
            case LEFT:
                lockRotateState = LockRotateState.UP;
                break;
            case UP:
                lockRotateState = LockRotateState.RIGHT;
                break;
            case OFF:
                System.out.println("Trying to rotate clockwise while auto lock off");
                break;
        }
    }

    public void lockRotateCounterclockwise() {
        switch (lockRotateState) {
            case RIGHT:
                lockRotateState = LockRotateState.UP;
                break;
            case UP:
                lockRotateState = LockRotateState.LEFT;
                break;
            case LEFT:
                lockRotateState = LockRotateState.DOWN;
                break;
            case DOWN:
                lockRotateState = LockRotateState.RIGHT;
                break;
            case OFF:
                System.out.println("Trying to rotate counterclockwise while auto lock off");
                break;
        }
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
        userGyroOffset = Rotation2d.fromDegrees(0).minus(getYaw()); // makes the front of the robot for the driver the current front of the robot
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

    public void forceAcceptNextVision() {
        forceAcceptNextVision = true;
    }

    public void updatePoseEstimator(Pose2d pose, double latency) { // updates the pose estimator from vision
        if (forceAcceptNextVision) {
            resetOdometry(pose);
            forceAcceptNextVision = false;
            return;
        }
        if (pose.getTranslation().getDistance(getPose().getTranslation()) < Constants.VisionConstants.maximumOffset) // throws out bad vision data
            poseEstimator.addVisionMeasurement(pose, latency);
        else
            System.out.println("Bad vision data recieved; distance: " + pose.getTranslation().getDistance(getPose().getTranslation()) + "; latency: " + latency);
    }

    public void setSwerveState(TeleopState newState) {
        teleopState = newState;
    }

    public static enum TeleopState {
        NORMAL,
        LOCKED, // for defense, makes an x formation
        AIMBOT
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (isFirstPath) { this.resetOdometry(traj.getInitialHolonomicPose()); }
            }),
            new PPSwerveControllerCommand(
                traj, 
                this::getPose, 
                Constants.Swerve.swerveKinematics, 
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
                this::setModuleStates,
                false,
                this
            )
        );
    }

    public Command aimbot() {
        Pose2d targetPose = FieldRegion.lookup(getPose()).getTargetPose();

        if (targetPose == null) {
            DriverStation.reportWarning("TARGET NOT FOUND", false);
            return null;
        }

        return new SequentialCommandGroup(
            new InstantCommand(() -> resetModulesToAbsolute()),
            new WaitCommand(0.1),
            followTrajectoryCommand(generateStraightTrajectory(getPose(), targetPose), false),
            new InstantCommand(() -> resetModulesToAbsolute()),
            new WaitCommand(0.1)
        );
    }

    public static PathPlannerTrajectory generateStraightTrajectory(Pose2d initialPose, Pose2d targetPose) {
        return PathPlanner.generatePath( // TODO: account for initial velocity
            Constants.AutoConstants.kPathConstraints, 
            new PathPoint(initialPose.getTranslation(), getHeading(initialPose, targetPose), initialPose.getRotation()),
            new PathPoint(targetPose.getTranslation(), getHeading(targetPose, initialPose), targetPose.getRotation())
        );
    }

    public static Rotation2d getHeading(Pose2d one, Pose2d two) {
        return Rotation2d.fromRadians(
            Math.atan2(two.getY() - one.getY(), two.getX() - one.getX())
        );
    }
}

