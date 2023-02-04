package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PincerArm;

public class RunPincerArm extends CommandBase {
  private PincerArm pincerArm;
  public RunPincerArm(PincerArm pincerArm) {
    this.pincerArm = pincerArm;
    addRequirements(pincerArm);
  }

  @Override
  public void execute() {
    pincerArm.runPincerArm();
  }

  @Override
  public void end(boolean interrupted) {
    pincerArm.stop();
  }

  public boolean isFinished() {
    return true;
  }
}