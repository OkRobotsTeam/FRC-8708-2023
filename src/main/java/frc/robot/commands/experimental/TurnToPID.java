package frc.robot.commands.experimental;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class TurnToPID extends CommandBase{

    private final double m_heading;
    private final double m_speed;
    private final Drivetrain m_drivetrain;
    private final ADIS16470_IMU m_gyro;

    private double startingHeading;

    private double error;

    private final double kP = 0.02;

    public TurnToPID(double heading, double maxSpeed, Drivetrain drivetrain) {
        m_heading = heading % 360;
        m_speed = maxSpeed;
        m_drivetrain = drivetrain;
        m_gyro = drivetrain.gyro;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.setRampRate(0.4);

        startingHeading = m_gyro.getAngle() % 360;

        double turnRight = (m_heading - startingHeading)%360;
        if (turnRight<0) {turnRight+=360;}
        double turnLeft = (startingHeading - m_heading)%360;
        if (turnLeft<0) {turnLeft+=360;}

        m_drivetrain.setBrakeMode(true);

        System.out.println("Starting turn to "+m_heading);

    }

    public double clamp(double in, double m) {
        return (in>m?m:(in<-m?-m:in));
    }

    @Override
    public void execute() {
        double currHeading = m_gyro.getAngle();
        double dRight = (m_heading - currHeading)%360;
        while (dRight<0) {dRight += 360;}
        double dLeft = (currHeading - m_heading)%360;
        while (dLeft<0) {dLeft += 360;}
        error = Math.min(dRight,dLeft);

        System.out.println(error);

        double velocity = -clamp(error * kP, m_speed);

        // STUPID SIGNS WILL BE THE END OF ME I SWEAR
        if (dLeft < dRight) {
            // turn left because still on track
            m_drivetrain.tankDriveRawCorrectDirection(-velocity, velocity, false);
            System.out.println("LEFT");
        } else {
            // turn right because overshot
            m_drivetrain.tankDriveRawCorrectDirection(velocity, -velocity, false);
            System.out.println("RIGHT");
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(error)<=DriveConstants.kAllowableHeadingOffset);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.tankDriveRawCorrectDirection(0, 0, false);
        System.out.println("Done, current heading: " + m_gyro.getAngle() + " off by "+error);
        m_drivetrain.targetHeading = m_heading;
    }
}
