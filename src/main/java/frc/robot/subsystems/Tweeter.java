package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Tweeter extends CommandBase {
    private Orchestra orchestra;

    public Tweeter(String chrpFile) {
        this.orchestra = new Orchestra();
        orchestra.loadMusic(Filesystem.getDeployDirectory().toPath().resolve(chrpFile).toString());
    }

    @Override
    public void initialize() {
        List<TalonFX> instruments = RobotContainer.rotatingArm.getMotors();
        for (TalonFX instrument : instruments) {
            orchestra.addInstrument(instrument);
            instrument.set(TalonFXControlMode.MusicTone, 0);
        }
        orchestra.play();
    }

    @Override
    public void end(boolean interrupted) {
        orchestra.stop();
    }
}