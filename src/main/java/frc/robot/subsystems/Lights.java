package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    
    public static final double violet = 0.91;
    public static final double yellow = 0.69;
    public static final double off = 0.99;
    public static final double redchaser = -0.31;
    public static final double bluechaser = 

    public final PWMSparkMax LED = new PWMSparkMax(0);

    public Lights() {
        LED.set(redchaser);
    }

    public void setViolet() {
        LED.set(violet);
    }

    public void setYellow() {
        LED.set(yellow);
    }

    public void setChaser() {
        LED.set(redchaser);
    }

    public void setOff() {
        LED.set(off);
    }
    
}
