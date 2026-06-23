package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;

public class dioPortTest extends TimedRobot{
    private final DigitalInput dio0 = new DigitalInput(0);
    private final DigitalInput dio1 = new DigitalInput(1);
    private final DigitalInput dio2 = new DigitalInput(2);

    @Override
    public void robotPeriodic() {
        // Print RAW values directly to the RioLog console
        System.out.println("P0: " + dio0.get() + " | P1: " + dio1.get() + " | P2: " + dio2.get());
    }
}
