package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PincerArm extends SubsystemBase {
  private PneumaticHub pH;
  private DoubleSolenoid dS;

  private boolean extended = false;

  public PincerArm(){
    pH = new PneumaticHub(1);
    dS = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 1);
    pH.enableCompressorDigital();
    dS.set(Value.kReverse);
  }

  public void runPincerArm(){
    extended = !extended;
    dS.set((extended) ? Value.kForward : Value.kReverse);
  }

  public void stop(){
    dS.set(Value.kOff);
  }
}