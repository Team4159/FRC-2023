package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotatingArm;
import frc.robot.subsystems.RotatingArm.ArmState;

public class SetRotatingArmState extends CommandBase {
    private RotatingArm rotatingArm;
    private ArmState armState;

    public SetRotatingArmState(RotatingArm rotatingArm, ArmState armState) {
        this.rotatingArm = rotatingArm;
        this.armState = armState;
        addRequirements(rotatingArm);
    }

    @Override
    public void initialize() {
        rotatingArm.setArmState(armState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
