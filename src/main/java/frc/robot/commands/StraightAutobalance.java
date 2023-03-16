package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.LockRotateState;

public class StraightAutobalance extends CommandBase {
    private Swerve s_Swerve;

    private PIDController pidController;

    private AutobalancePhase phase;

    private AutobalanceDirection direction;

    private int directionVal;

    private Alliance alliance;

    public StraightAutobalance(AutobalanceDirection direction) {
        s_Swerve = RobotContainer.s_Swerve;

        pidController = new PIDController(AutoConstants.kPautobalance, AutoConstants.kIautobalance, AutoConstants.kDautobalance);
        pidController.setTolerance(AutoConstants.autoBalanceTolerence);

        phase = AutobalancePhase.RAM;

        alliance = DriverStation.getAlliance();
        this.direction = direction;

        switch (alliance) {
            case Blue:
                directionVal = (this.direction.equals(AutobalanceDirection.IN)) ? 1 : -1;
                break;
            case Red:
                directionVal = (this.direction.equals(AutobalanceDirection.IN)) ? -1 : 1;
                break;
            case Invalid:
                directionVal = 0;
                System.out.println("Invalid allaince");
                break;
        }

        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        double maxSpeed = 0;
        double pitch = s_Swerve.getPitch();
        double translationVal = 0;

        switch (phase) {
            case RAM:
                maxSpeed = AutoConstants.maxAutobalanceSpeed;
                translationVal = 1;
                if (pitch >= 8) phase = AutobalancePhase.UPHILL;
                break;
            case UPHILL:
                maxSpeed = AutoConstants.maxAutobalanceUphillSpeed;
                translationVal = 1;
                if (pitch <= -3) phase = AutobalancePhase.REVERSE;
                break;
            case REVERSE:
                maxSpeed = AutoConstants.maxAutobalanceReverseSpeed;
                translationVal = calculatePID(pitch);
                if (pidController.atSetpoint()) phase = AutobalancePhase.UPHILL;
                break;
            case FINISH:
                return;
        }
        

        s_Swerve.straightBalance(
            new Translation2d(translationVal, 0).times(maxSpeed).times(directionVal), 
            LockRotateState.UP,
            maxSpeed
        );
    }

    @Override
    public boolean isFinished() {
        return phase == AutobalancePhase.FINISH;
    }

    @Override
    public void end(boolean i) {
        s_Swerve.straightBalance(
            new Translation2d(0, 0), 
            alliance.equals(Alliance.Blue) ? LockRotateState.LEFT : LockRotateState.RIGHT,
            0
        );
    }

    public double calculatePID(double pitch) {
        return pidController.calculate(pitch, 0);
    }

    public static enum AutobalancePhase {
        RAM,
        UPHILL,
        REVERSE,
        FINISH
    }

    public static enum AutobalanceDirection {
        IN, //autobalance from the grid toward the center of the field
        OUT //autobalance from the center of the field toward the grid
    }
}
