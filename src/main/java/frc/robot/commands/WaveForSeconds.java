package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class WaveForSeconds extends CommandBase {

    private final double m_seconds;
    private final double m_wave_duration;
    private final double m_arm_position_1;
    private final double m_arm_position_2;
    private final Arm m_arm;

    private double m_start_time;
    private double m_current_time;

    private double time_since_start() {
        return (System.currentTimeMillis() - m_start_time) / 1000.0d ;
    }


    public WaveForSeconds(double seconds, double waves_per_second, double arm_position_1, double arm_position_2, Arm arm) {
        m_seconds = seconds;
        m_wave_duration = (1.0f / waves_per_second);
        m_arm_position_1 = arm_position_1;
        m_arm_position_2 = arm_position_2;
        m_arm = arm;
        addRequirements(arm);
    }

    private double linear_interpolation(double a, double b, double f)
    {
        /* Return a value that is f of the wy from a to b
         * ie: for inputs a:5, b:10, f:0.5, return a value half way between 5 and 10
         * the above example should return 7.5
         */

        return a * (1.0 - f) + (b * f);
    }

    @Override
    public void initialize() {
        m_current_time = System.currentTimeMillis();
        m_start_time = m_current_time;
        if (!m_arm.getPistonRaised()) {
            m_arm.setPistonRaised(true);
        }

    }

    @Override
    public void execute() {
        double fraction = Math.sin(2.0d * (time_since_start() * (Math.PI / m_wave_duration)));
        fraction = (fraction + 1.0d) / 2.0d;
        double target_position = linear_interpolation(m_arm_position_1, m_arm_position_2, fraction);
        m_arm.setElbowSetpoint(target_position);
    }

    @Override
    public boolean isFinished() {
        if (!m_arm.getPistonRaised()) {
            m_arm.setElbowExtended(false);
            return true;
        }
        
        System.out.println("current_time is: " + time_since_start());
        System.out.println("end_time is: " + m_seconds);
        return (time_since_start() > m_seconds);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setElbowExtended(false);
    }
}
