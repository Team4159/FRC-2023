package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.WheeledIntakeConstants.WheeledIntakeState;
import frc.robot.RobotContainer;
import frc.robot.commands.StraightAutobalance.AutobalanceDirection;
import frc.robot.subsystems.CascadingArm;
import frc.robot.subsystems.RotatingArm;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.StateController.GameElementState;
import frc.robot.subsystems.StateController.PositionState;
import frc.robot.subsystems.WheeledIntake;
import frc.robot.subsystems.Wrist;

// Container class to hold lots of commands for auto
public class AutoCommands {

    // variables to hold subsystem pointers
    private StateController stateController = RobotContainer.stateController;
    private RotatingArm rotatingArm = RobotContainer.rotatingArm;
    private CascadingArm cascadingArm = RobotContainer.cascadingArm;
    private Wrist wrist = RobotContainer.wrist; 
    private WheeledIntake wheeledIntake = RobotContainer.wheeledIntake;

    // sequential command to score cubes in low
    public Command autoCubeLow() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE_CUBE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    // sequential command to score cubes in mid
    public Command autoCubeMid() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitUntilCommand(cascadingArm::atDebouncedSetPoint).withTimeout(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE_CUBE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    // sequential command to score cubes in high
    public Command autoCubeHigh() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitUntilCommand(cascadingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitUntilCommand(wrist::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE_CUBE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }
    
    // sequential command to score cones in low UNTESTED
    public Command autoConeLow() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE_CUBE)), // intentionally cube for more power when low
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    // sequential command to score cones in mid UNTESTED
    public Command autoConeMid() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitUntilCommand(cascadingArm::atDebouncedSetPoint).withTimeout(1),
            new InstantCommand(() -> stateController.setForceScore(true)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE_CONE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setForceScore(false)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    // sequential command to score cones in high UNTESTED
    public Command autoConeHigh() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitUntilCommand(cascadingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setForceScore(true)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE_CONE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setForceScore(false)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    // sequential command to intake cubes from ground
    public Command groundIntakeCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.GROUND_INTAKING)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.INTAKE_CUBE))
        );
    }

    // sequential command to intake cones from ground
    public Command groundIntakeCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.GROUND_INTAKING)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.INTAKE_CONE))
        );
    }

    // sequential command to tuck cubes -- for use in auto after intaking
    public Command tuckCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED)),
            new WaitCommand(1)
        );
    }

    // sequential command to tuck cones -- for use in auto after intaking
    public Command tuckCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED)),
            new WaitCommand(1)
        );
    }


    // autobalances toward center of field
    public Command autobalanceIn() {
        return new StraightAutobalance(AutobalanceDirection.IN);
    }

    // autobalances away from center of field UNTESTED
    public Command autobalanceOut() {
        return new StraightAutobalance(AutobalanceDirection.OUT);
    }
}
