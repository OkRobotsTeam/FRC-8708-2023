package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveForTick extends CommandBase {

    private final double m_distance;
    private final double m_targetHeading;
    private final double m_speed;
    private final Drivetrain m_drive;
    private final double cmPerRot;
    private final boolean m_brake;
    private double delta_heading;
    private final double kDecelrationRate = 1.0; //motor power per second 
    private int m_rampUpTicks;
    private int m_rampDownTicks;
    private int m_tickNumber;
    private double m_distanceTraveled;
    private double start_pos;
    private int m_decelerationStartTick;
    private double m_maxDistancePerTick;
    private double m_calculatedSpeedPerTickPerUnitPower;
    private double m_calibrationTotalPower;

    public DriveForTick(double heading, double distance_in, double unsigned_speed, Drivetrain drive, boolean brake, int rampUpTicks, int rampDownTicks) {
        m_targetHeading = heading;
        m_distance = distance_in;
        if (distance_in < 0) {
            m_speed = -unsigned_speed;
        } else {
            m_speed = unsigned_speed;
        }
        m_drive = drive;
        m_brake = brake;
        m_rampUpTicks = rampUpTicks;
        m_rampDownTicks = rampDownTicks;
        m_tickNumber = 0;
        m_distanceTraveled = 0;
        m_decelerationStartTick = 0;
        cmPerRot = DriveConstants.kSlowRevPerRot * DriveConstants.kWheelCircumference;
        m_calibrationTotalPower = 0;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        start_pos = m_drive.getAvgEncoder();
        m_drive.tankDriveRaw(0, 0, false);
        // System.out.println("DISTANCE TO GO: " + m_distance);
        m_drive.setBrakeMode(m_brake);
        m_drive.setRampRate(0.5);
    }

    @Override
    public void execute() {
        double currentHeading = m_drive.gyro.getAngle() % 360;
        double leftTurnDifference = (currentHeading - m_targetHeading);
        double rightTurnDifference = (m_targetHeading - currentHeading);
        if (leftTurnDifference < 0) {
            leftTurnDifference += 360;
        }
        if (rightTurnDifference < 0) {
            rightTurnDifference += 360;
        }
       
        double distanceTraveled = Math.abs(m_drive.getAvgEncoder()-start_pos) * cmPerRot;
        double distanceRemaining = Math.abs(m_distance) - distanceTraveled;
        m_tickNumber++;
        double distanceLastTick = distanceTraveled - m_distanceTraveled;
        m_distanceTraveled=distanceTraveled;
        
        
        double targetSpeed = accelerationCurve(m_speed, distanceTraveled, distanceRemaining, m_tickNumber, distanceLastTick );
        m_calibrationTotalPower += targetSpeed;
        if (Math.abs(leftTurnDifference) < Math.abs(rightTurnDifference)) {
            delta_heading = leftTurnDifference;
            m_drive.tankDriveRaw((delta_heading * -DriveConstants.kCorrectionAggression) - m_speed, (delta_heading * DriveConstants.kCorrectionAggression) - m_speed, false);
        } else {
            delta_heading = rightTurnDifference;
            m_drive.tankDriveRaw((delta_heading * DriveConstants.kCorrectionAggression) - m_speed, (delta_heading * -DriveConstants.kCorrectionAggression) - m_speed, false);
        }
        
    }

    
    private double accelerationCurve(double speed, double distanceTraveled, double distanceRemaining, int tickNumber, double distancePerTick) {
        if (m_decelerationStartTick > 0) {
            //slowing
            int ticksRampedDown = tickNumber - m_decelerationStartTick;
            int ticksRemaining = m_rampDownTicks - ticksRampedDown;
            double desiredSpeedPerTick = 0;
            if (ticksRemaining == 0) {
                return(0.0);
            } else if (ticksRemaining == 1) {
                desiredSpeedPerTick = distanceRemaining;
            } else {
                desiredSpeedPerTick = distanceRemaining/ (ticksRemaining) * 2.3;
            }
            double distancePerPower = m_distanceTraveled / m_calibrationTotalPower;
            return (desiredSpeedPerTick / distancePerPower);
        } else if (tickNumber < m_rampUpTicks) {
            //accelerating
            if ( (distancePerTick * m_rampDownTicks / 2) > distanceRemaining) {
                m_decelerationStartTick=tickNumber;
            }

            return (m_speed *( tickNumber / m_rampUpTicks));
        } else {
            //at_speed
            if ( (distancePerTick * m_rampDownTicks / 2) > distanceRemaining) {
                m_decelerationStartTick=tickNumber;
            }
            return m_speed;
        }    
    }

    @Override
    public boolean isFinished() {
        System.out.println("Drive For Tick finished in " + m_tickNumber + " ticks Deceleration Start Tick " + m_decelerationStartTick + " Ramp Down Ticks " + m_rampDownTicks + " Distance traveled " + m_distanceTraveled);
        return (m_rampDownTicks - m_tickNumber - m_decelerationStartTick < 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setRampRate(0);
        m_drive.tankDriveRaw(0, 0, false);
        System.out.println("DONE, DEGREES OFF COURSE: " + Math.abs(delta_heading));
    }
}
