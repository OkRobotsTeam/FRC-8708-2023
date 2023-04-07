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
    private double delta_heading_deg;
    private int m_rampUpTicks;
    private int m_rampDownTicks;
    private int m_tickNumber = 0;
    private double m_distanceTraveled;
    private double m_avgEncoderStartPosition;
    private int m_decelerationStartTick;
    private double m_calibrationTotalPower;
    private enum AccelerationState {ACCELERATING, AT_SPEED, SLOWING, DONE};
    private AccelerationState accelerationState;
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
        inPerRot = DriveConstants.kSlowRevPerRot * DriveConstants.kWheelCircumference;
        
        addRequirements(drive);
    }

    @Override 
    public void initialize() {
        m_calibrationTotalPower = 0;
        m_avgEncoderStartPosition = m_drive.getAvgEncoder();
        m_tickNumber = 0;
        m_distanceTraveled = 0;
        m_decelerationStartTick = 0;
        m_drive.setBrakeMode(m_brake);
        System.out.println("Start Position" + m_avgEncoderStartPosition);
        if (m_rampUpTicks > 0 ) {
            accelerationState = AccelerationState.AT_SPEED;
        } else {
            accelerationState = AccelerationState.ACCELERATING;
        }
    }


    @Override
    public void execute() {

       System.out.println("averageEncoder: " + m_drive.getAvgEncoder()+ "aeStartPosition: " + m_avgEncoderStartPosition);
        double distanceTraveled = Math.abs(m_drive.getAvgEncoder()-m_avgEncoderStartPosition) * inPerRot;
        double distanceRemaining = Math.abs(m_targetDistance_in) - distanceTraveled;
        m_tickNumber++;
        double distanceLastTick = distanceTraveled - m_distanceTraveled;
        m_distanceTraveled=distanceTraveled;
        
        System.out.println("distanceTraveled: " + distanceTraveled + "distanceRemaining:" + distanceRemaining);

        double calculatedSpeed = accelerationCurve(m_targetSpeed, distanceTraveled, distanceRemaining, distanceLastTick);
        System.out.println("acceleration curve output: "+calculatedSpeed);

        m_calibrationTotalPower += calculatedSpeed;
        double turnFactor = getTurnFactor();

        System.out.println("TargetSpeed: " + calculatedSpeed + " turnFactor: " + turnFactor);
        m_drive.tankDriveRaw(calculatedSpeed - turnFactor, calculatedSpeed + turnFactor,false);
        
    }

    private double getTurnFactor() {
        double currentHeading_deg = m_drive.gyro.getAngle() % 360;
        double leftTurnDifference = (currentHeading_deg - m_targetHeading_deg);
        double rightTurnDifference = (m_targetHeading_deg - currentHeading_deg);
        if (leftTurnDifference < 0) {
            leftTurnDifference += 360;
        }
        if (rightTurnDifference < 0) {
            rightTurnDifference += 360;
        }
        if (Math.abs(leftTurnDifference) < Math.abs(rightTurnDifference)) {
            delta_heading_deg = leftTurnDifference;
            return (delta_heading_deg * -DriveConstants.kCorrectionAggression);
        } else {
            delta_heading_deg = rightTurnDifference;
            return (delta_heading_deg * DriveConstants.kCorrectionAggression);
        }
    }

    private double accelerationCurve(double speed, double distanceTraveled, double distanceRemaining, double distancePerTick) {
        
        if (accelerationState == AccelerationState.ACCELERATING) {
            //accelerating
            System.out.println("accellerating " + m_tickNumber * m_rampUpTicks);

            if ( (distancePerTick * m_rampDownTicks / 2) > distanceRemaining) {
                m_decelerationStartTick=m_tickNumber;
            }

            return (m_targetSpeed * ( m_tickNumber / (double) m_rampUpTicks ) );
        } else if (accelerationState == AccelerationState.SLOWING) {
            //slowing
            System.out.println("Slowing "+(m_tickNumber-m_decelerationStartTick) + "of" + m_rampDownTicks);
            int ticksRampedDown = m_tickNumber - m_decelerationStartTick;
            int ticksRemaining = m_rampDownTicks - ticksRampedDown;
            double desiredSpeedPerTick = 0;
            if (distanceRemaining < 0) {
                return(0.0);
            }
            if (ticksRemaining <= 0) {
                return(0.0);
            } else if (ticksRemaining == 1) {
                desiredSpeedPerTick = distanceRemaining;
            } else {
                desiredSpeedPerTick =( distanceRemaining/ ticksRemaining) * 2.3;
            }
            double distancePerPower = m_distanceTraveled / m_calibrationTotalPower;
            return (desiredSpeedPerTick / distancePerPower);

        } else if (accelerationState == AccelerationState.AT_SPEED) {
            //at_speed
            System.out.println("at speed");

            if ( (distancePerTick * m_rampDownTicks / 2) > distanceRemaining) {
                m_decelerationStartTick=m_tickNumber;
            }
            return m_targetSpeed;
        }    else {
            return (0.0);
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
        System.out.println("DONE, DEGREES OFF COURSE: " + Math.abs(delta_heading_deg));
    }
}
