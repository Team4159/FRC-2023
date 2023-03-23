package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.WheeledIntakeConstants.WheeledIntakeState;
import frc.robot.commands.StraightAutobalance.AutobalanceDirection;
import frc.robot.subsystems.CascadingArm;
import frc.robot.subsystems.RotatingArm;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.WheeledIntake;
import frc.robot.subsystems.StateController.GameElementState;
import frc.robot.subsystems.StateController.PositionState;

public class AutoCommands {

    private StateController stateController = RobotContainer.stateController;
    private RotatingArm rotatingArm = RobotContainer.rotatingArm;
    private CascadingArm cascadingArm = RobotContainer.cascadingArm;
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
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitUntilCommand(cascadingArm::atDebouncedSetPoint).withTimeout(1),
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
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitUntilCommand(cascadingArm::atDebouncedSetPoint).withTimeout(1),
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
    
    public ProxyCommand autoCubeSelect(IntSupplier sup) {

        if (sup.getAsInt() == 1) {
            return autoCubeLow().asProxy();
        }
        else if (sup.getAsInt() == 2) {
            return autoCubeMid().asProxy();
        }
        else if (sup.getAsInt() == 3) {
            return autoCubeHigh().asProxy();
        }

        return new PrintCommand("no auto height selected").asProxy();
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
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitUntilCommand(cascadingArm::atDebouncedSetPoint).withTimeout(1),
            new InstantCommand(() -> stateController.setForceScore(true)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.NEUTRAL)),
            new InstantCommand(() -> stateController.setForceScore(false)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.LOW_SCORE)),
            new WaitCommand(1),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED))
        );
    }

    public Command autoConeHigh() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.MID_SCORE)),
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitUntilCommand(rotatingArm::atDebouncedSetPoint).withTimeout(1.5),
            new InstantCommand(() -> stateController.setPositionState(PositionState.HIG_SCORE)),
            new WaitUntilCommand(cascadingArm::atDebouncedSetPoint).withTimeout(1),
            new InstantCommand(() -> stateController.setForceScore(true)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.OUTTAKE)),
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

    public ProxyCommand autoConeSelect(IntSupplier sup) {
        if (sup.getAsInt() == 1) {
            return autoConeLow().asProxy();
        }
        else if (sup.getAsInt() == 2) {
            return autoConeMid().asProxy();
        }
        else if (sup.getAsInt() == 3) {
            return autoConeHigh().asProxy();
        } 
        
        return new PrintCommand("no auto height selected").asProxy();
    }



    public Command groundIntakeCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.GROUND_INTAKING)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.INTAKE))
        );
    }

    public Command groundIntakeCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.GROUND_INTAKING)),
            new WaitCommand(1),
            new InstantCommand(() -> wheeledIntake.setWheeledIntakeState(WheeledIntakeState.INTAKE))
        );
    }

    public Command tuckCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CUBE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED)),
            new WaitCommand(1)
        );
    }

    public Command tuckCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> stateController.setGameElementState(GameElementState.CONE)),
            new InstantCommand(() -> stateController.setPositionState(PositionState.TUCKED)),
            new WaitCommand(1)
        );
    }



    public Command autobalanceIn() {
        return (new AutoRotateInPlace(180).asProxy()).andThen(new StraightAutobalance(AutobalanceDirection.IN));
    }

    public Command autobalanceOut() {
        return (new AutoRotateInPlace(180).asProxy()).andThen(new StraightAutobalance(AutobalanceDirection.IN));
    }
}
