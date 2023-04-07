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

    private boolean isLeft;

    private double startingHeading;

    private double error;

    private final double kP = 0.1;

    public TurnToPID(double heading, double maxSpeed, Drivetrain drivetrain) {
        m_heading = heading % 360;
        m_speed = maxSpeed;
        m_drivetrain = drivetrain;
        m_gyro = drivetrain.gyro;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startingHeading = m_gyro.getAngle() % 360;

        double turnRight = (m_heading - startingHeading)%360;
        double turnLeft = (startingHeading - m_heading)%360;

        isLeft = (turnLeft < turnRight);

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
        double dLeft = (currHeading - m_heading)%360;
        error = Math.min(dRight,dLeft);

        double velocity = clamp(error * kP, m_speed);

        // STUPID SIGNS WILL BE THE END OF ME I SWEAR
        if (isLeft) {
            // we are turning left
            if (dLeft < dRight) {
                // turn left because still on track
                m_drivetrain.tankDriveRawCorrectDirection(-velocity, velocity, false);
            } else {
                // turn right because overshot
                m_drivetrain.tankDriveRawCorrectDirection(velocity, -velocity, false);
            }
        } else {
            // we are turning right
            if (dRight < dLeft) {
                // turn right because still on track
                m_drivetrain.tankDriveRawCorrectDirection(velocity, -velocity, false);
            } else {
                // turn left because overshot
                m_drivetrain.tankDriveRawCorrectDirection(-velocity, velocity, false);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (error<=DriveConstants.kAllowableHeadingOffset);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.tankDriveRawCorrectDirection(0, 0, false);
        System.out.println("Done, current heading: " + m_gyro.getAngle() + " off by "+error);
        m_drivetrain.targetHeading = m_heading;
    }
}
