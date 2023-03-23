package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class AutoRotateInPlace extends CommandBase {
    private Swerve s_Swerve;
    private PIDController pidController;

    private Debouncer debouncer;

    private double initialrot;
    private double goalRot;

    public AutoRotateInPlace(double rotation) { // rotation ccw +   cw -
        s_Swerve = RobotContainer.s_Swerve;
        pidController = new PIDController(AutoConstants.kPThetaController/100, 0, 0);
        pidController.enableContinuousInput(-180, 180);

        debouncer = new Debouncer(0.2);

        initialrot = s_Swerve.getYaw().getDegrees();
        goalRot = initialrot + rotation;
    }

    @Override
    public void execute() {
        s_Swerve.autoSpinInPlace(pidController.calculate(s_Swerve.getYaw().getDegrees(), goalRot));
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(Math.abs(s_Swerve.getYaw().getDegrees() - goalRot) < 5);
    }
    
}
