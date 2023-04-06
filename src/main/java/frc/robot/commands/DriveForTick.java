package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveForTick extends CommandBase {

    private final double m_targetDistance_in;
    private final double m_targetHeading_deg;
    private final double m_targetSpeed;
    private final Drivetrain m_drive;
    private final double inPerRot;
    private final boolean m_brake;
    private double delta_heading;
    private int m_rampUpTicks;
    private int m_rampDownTicks;
    private int m_tickNumber = 0;
    private double m_distanceTraveled;
    private double m_avgEncoderStartPosition;
    private int m_decelerationStartTick;
    private double m_calibrationTotalPower;

    public DriveForTick(double heading, double distance_in, double unsigned_speed, Drivetrain drive, boolean brake, int rampUpTicks, int rampDownTicks) {
        m_targetHeading_deg = heading;
        m_targetDistance_in = distance_in;
        if (distance_in < 0) {
            m_targetSpeed = -unsigned_speed;
        } else {
            m_targetSpeed = unsigned_speed;
        }
        m_drive = drive;
        m_brake = brake;
        m_rampUpTicks = rampUpTicks;
        m_rampDownTicks = rampDownTicks;
        m_tickNumber = 0;
        m_distanceTraveled = 0;
        m_decelerationStartTick = 0;
        inPerRot = DriveConstants.kSlowRevPerRot * DriveConstants.kWheelCircumference;
        m_calibrationTotalPower = 0;
        m_avgEncoderStartPosition = m_drive.getAvgEncoder();
        m_drive.setBrakeMode(m_brake);
        addRequirements(drive);
    }


    @Override
    public void execute() {
        double currentHeading_deg = m_drive.gyro.getAngle() % 360;
        double leftTurnDifference = (currentHeading_deg - m_targetHeading_deg);
        double rightTurnDifference = (m_targetHeading_deg - currentHeading_deg);
        if (leftTurnDifference < 0) {
            leftTurnDifference += 360;
        }
        if (rightTurnDifference < 0) {
            rightTurnDifference += 360;
        }
       
        double distanceTraveled = Math.abs(m_drive.getAvgEncoder()-m_avgEncoderStartPosition) * inPerRot;
        double distanceRemaining = Math.abs(m_targetDistance_in) - distanceTraveled;
        m_tickNumber++;
        double distanceLastTick = distanceTraveled - m_distanceTraveled;
        m_distanceTraveled=distanceTraveled;
        
        
        double targetSpeed = accelerationCurve(m_targetSpeed, distanceTraveled, distanceRemaining, distanceLastTick);
        m_calibrationTotalPower += targetSpeed;
        if (Math.abs(leftTurnDifference) < Math.abs(rightTurnDifference)) {
            delta_heading = leftTurnDifference;
            m_drive.tankDriveRaw((delta_heading * -DriveConstants.kCorrectionAggression) - m_targetSpeed, (delta_heading * DriveConstants.kCorrectionAggression) - m_targetSpeed, false);
        } else {
            delta_heading = rightTurnDifference;
            m_drive.tankDriveRaw((delta_heading * DriveConstants.kCorrectionAggression) - m_targetSpeed, (delta_heading * -DriveConstants.kCorrectionAggression) - m_targetSpeed, false);
        }
        
    }

    
    private double accelerationCurve(double speed, double distanceTraveled, double distanceRemaining, double distancePerTick) {
        if (m_decelerationStartTick > 0) {
            //slowing
            int ticksRampedDown = m_tickNumber - m_decelerationStartTick;
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
        } else if (m_tickNumber < m_rampUpTicks) {
            //accelerating
            if ( (distancePerTick * m_rampDownTicks / 2) > distanceRemaining) {
                m_decelerationStartTick=m_tickNumber;
            }

            return (m_targetSpeed * (m_rampUpTicks!=0 ? m_tickNumber / m_rampUpTicks : 1) );
        } else {
            //at_speed
            if ( (distancePerTick * m_rampDownTicks / 2) > distanceRemaining) {
                m_decelerationStartTick=m_tickNumber;
            }
            return m_targetSpeed;
        }    
    }

    @Override
    public boolean isFinished() {
        if (m_decelerationStartTick > 0) {
            if (m_tickNumber >= m_decelerationStartTick+m_rampDownTicks) {
                System.out.println("Drive For Tick finished in " + m_tickNumber + " ticks Deceleration Start Tick " + m_decelerationStartTick + " Ramp Down Ticks " + m_rampDownTicks + " Distance traveled " + m_targetDistance_in);
                return(true);
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setRampRate(0);
        m_drive.tankDriveRaw(0, 0, false);
        System.out.println("DONE, DEGREES OFF COURSE: " + Math.abs(delta_heading));
    }
}
