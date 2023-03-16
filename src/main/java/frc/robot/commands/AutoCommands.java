package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.WheeledIntakeConstants.WheeledIntakeState;
import frc.robot.commands.StraightAutobalance.AutobalanceDirection;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.WheeledIntake;
import frc.robot.subsystems.StateController.GameElementState;
import frc.robot.subsystems.StateController.PositionState;

public class AutoCommands {

    private StateController stateController = RobotContainer.stateController;
    private WheeledIntake wheeledIntake = RobotContainer.wheeledIntake;

    public Command autoCubeLow() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    public Command autoCubeMid() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    public Command autoCubeHigh() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitCommand(1.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }
    

    
    public Command autoConeLow() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    public Command autoConeMid() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    public Command autoConeHigh() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitCommand(1.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    public Command autobalanceIn() {
        return new StraightAutobalance(AutobalanceDirection.IN);
    }

    public Command autobalanceOut() {
        return new StraightAutobalance(AutobalanceDirection.OUT);
    }
}
