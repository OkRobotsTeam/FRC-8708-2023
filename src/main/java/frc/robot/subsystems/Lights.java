package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

    

    public static final double violet = 0.91;
    public static final double yellow = 0.69;
    public static final double off = 0.99;
    public static final double redchaser = -0.31;
    public static final double bluechaser = -0.29;
    public static double teamcolor;

    public static boolean isRed;

    public final PWMSparkMax LED = new PWMSparkMax(0);

    public Lights() {
        teamChaser();
    }

    public void setViolet() {
        LED.set(violet);
    }

    public void setYellow() {
        LED.set(yellow);
    }

    public void blueChaser() {
        LED.set(bluechaser);
    }

    public void redChaser() {
        LED.set(redchaser);
    }

    public void setOff() {
        LED.set(off);
    }

    public void teamChaser() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) { // Might always return false (DriverStation.Alliance.Invalid) if robot is not plugged in to field on initialization?
            teamcolor = redchaser;
        } else {
            teamcolor = bluechaser;
        }
        LED.set(teamcolor);
    }

    public void init() {
        teamChaser();
    }
}
