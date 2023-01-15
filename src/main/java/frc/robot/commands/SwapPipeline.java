package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwapPipeline extends CommandBase {
    private int num;
    
    public SwapPipeline(int num) {
        this.num = num;
    }

    public void execute() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(num);
    }
}
