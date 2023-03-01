package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LED extends SubsystemBase {
    public enum LEDState {
        WHITE, BLACK, PURPLE, YELLOW,
        RAINBOW, NONBINARY, GENDERFLUID, GAY, LESBIAN, BI, TRANS
    }

    private Map<LEDState, Command> commands = new HashMap<LEDState, Command>();
    {
        commands.put(LEDState.WHITE, new InstantCommand(() -> setLED(75, 75, 75), this));
        commands.put(LEDState.PURPLE, new InstantCommand(() -> setLED(70, 0, 100), this));
        commands.put(LEDState.YELLOW, new InstantCommand(() -> setLED(150, 75, 0), this));
        commands.put(LEDState.BLACK, new InstantCommand(() -> setLED(0, 0, 0), this));
        commands.put(LEDState.RAINBOW, new ChromaLED(this, (double i) -> Color.fromHSV((int)Math.floor(i * 180), 255, 255)).repeatedly());
        commands.put(LEDState.NONBINARY, new LinearFlag(this, new int[]{
            0xFCF434, 0xFCFCFC, 0x9C59D1, 0x2C2C2C
        }).repeatedly());
        commands.put(LEDState.GENDERFLUID, new LinearFlag(this, new int[]{
            0xFF75A2, 0xFFFFFF, 0xBE18D6, 0x000000, 0x323DBC
        }).repeatedly());
        commands.put(LEDState.GAY, new LinearFlag(this, new int[]{
            0xE40303, 0xFF8C00, 0xFFED00, 0x008026, 0x004DFF, 0x750787
        }).repeatedly());
        commands.put(LEDState.LESBIAN, new LinearFlag(this, new int[]{
            0xD52D00, 0xEF7627, 0xFF9A56, 0xFFFFFF, 0xD362A4, 0xB85490, 0xA30262
        }).repeatedly());
        commands.put(LEDState.BI, new LinearFlag(this, new int[]{
            0xD60270, 0xD60270, 0x9B4F97, 0x0038A7, 0x0038A7
        }).repeatedly());
        commands.put(LEDState.TRANS, new LinearFlag(this, new int[]{
            0x5BCEFA, 0xF5A9B8, 0xFFFFFF, 0xF5A9B8, 0x5BCEFA
        }).repeatedly());
    };
    private AddressableLED strip;
    private AddressableLEDBuffer buffer;
    private LEDState state;

    public LED() { 
        strip = new AddressableLED(3);
        buffer = new AddressableLEDBuffer(300);
        strip.setLength(buffer.getLength());
        state = LEDState.BLACK;
        startLED();
    }

    public void setState(LEDState state) {
        getStateCommand().cancel();
        this.state = state;
        startLED();
    }

    public Command getStateCommand() {
        return commands.get(state);
    }

    public void startLED() {
        strip.start();
        getStateCommand().schedule();
    }

    public void stopLED() {
        getStateCommand().cancel();
        setLED(0, 0, 0);
        strip.stop();
    }

    public void setLED(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        strip.setData(buffer);
    }

    public static class BlinkLED extends SequentialCommandGroup {
        public BlinkLED(LED led){
            super(
                new InstantCommand(() -> led.stopLED()),
                new WaitCommand(0.5d),
                new InstantCommand(() -> led.startLED()),
                new WaitCommand(0.5d)
            );
        }
    }

    private static class ChromaLED extends CommandBase {
        private LED led;
        private LEDColorSupplier supplier;

        private ChromaLED(LED led, LEDColorSupplier supplier) {
            this.led = led;
            this.supplier = supplier;
            addRequirements(led);
        }

        @Override
        public void execute() {
            int len = led.buffer.getLength();
            int offset = (int)Math.floor((System.currentTimeMillis()/10) % len);
            for (int i = 0; i < len; i++)
                led.buffer.setLED((i+offset) % len, supplier.get((double)i/len));
            led.strip.setData(led.buffer);
        }

        public static interface LEDColorSupplier {
            public Color get(double progress);
        }
    }

    private static class LinearFlag extends ChromaLED {
        private static double reductionFactor = 0.9;
        private LinearFlag(LED led, int[] colors) {
            super(led, (double progress) -> {
                int color = colors[(int)Math.floor(progress*colors.length)];
                return new Color(
                    (int)Math.floor(((color >> 16)& 255) * reductionFactor),
                    (int)Math.floor(((color >> 8) & 255) * reductionFactor), 
                    (int)Math.floor(((color >> 0) & 255) * reductionFactor)
                );
            });
        }
    }
}