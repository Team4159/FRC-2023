package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

// Command to rotate in place a certain amount of degrees
public class AutoRotateInPlace extends CommandBase {
    private Swerve s_Swerve;
    private PIDController pidController;

    private Debouncer debouncer; // debouncer to make sure robot is not oscilating when command ends https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/debouncer.html

    private double initialrot;
    private double goalRot;

    public AutoRotateInPlace(double rotation) { // rotation ccw +   cw -
        s_Swerve = RobotContainer.s_Swerve;
        pidController = new PIDController(AutoConstants.kPThetaController/120, 0, 0); // initializes PID
        pidController.enableContinuousInput(-180, 180); // finds shortest path because circle

        debouncer = new Debouncer(AutoConstants.autoRotateDebounceTime);

        initialrot = s_Swerve.getYaw().getDegrees(); // grabs initial rotation from swerve subsystem
        goalRot = initialrot + rotation; // final rotation goal based on initial
    }

    @Override
    public void execute() {
        s_Swerve.autoSpinInPlace(pidController.calculate(s_Swerve.getYaw().getDegrees(), goalRot)); // runs swerve spin in place function continuously
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(Math.abs(s_Swerve.getYaw().getDegrees() - goalRot) < AutoConstants.autoRotateTolerance); // returns true when the debouncer is true for the set amount of time -> ends the command
    }
    
}
