package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwapVisionPipeline extends CommandBase {
    private int num;
    
    public SwapVisionPipeline(int num) {
        this.num = num;
    }

    @Override
    public void execute() { // send the new pipeline # to NetworkTables
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(num);
    }

    @Override
    public boolean isFinished() { // check whether the pipeline has in fact been changed
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getInteger(-1) == num;
    }
}
