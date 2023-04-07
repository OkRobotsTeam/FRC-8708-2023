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
    private double m_previousDistanceTraveled;
    private double m_avgEncoderStartPosition;
    private int m_slowingTick;
    private double m_calibrationTotalPower;
    double m_currentAverageEncoder;
    private enum AccelerationState {ACCELERATING, AT_SPEED, SLOWING, DONE};
    private AccelerationState m_accelerationState;
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
        m_currentAverageEncoder = m_avgEncoderStartPosition;
        m_tickNumber = 0;
        m_previousDistanceTraveled = 0;
        m_slowingTick = 0;
        m_drive.setBrakeMode(m_brake);
        System.out.println("Start Position" + m_avgEncoderStartPosition);
        if (m_rampUpTicks > 0 ) {
            m_accelerationState = AccelerationState.AT_SPEED;
        } else {
            m_accelerationState = AccelerationState.ACCELERATING;
        }
    }


    @Override
    public void execute() {
        double m_currentAverageEncoder = m_drive.getAvgEncoder();

       System.out.println("averageEncoder: " + m_currentAverageEncoder+ "aeStartPosition: " + m_avgEncoderStartPosition);
        double distanceTraveled = Math.abs(m_currentAverageEncoder-m_avgEncoderStartPosition) * inPerRot;
        double distanceRemaining = Math.abs(m_targetDistance_in) - distanceTraveled;
        m_tickNumber++;
        double distanceLastTick = distanceTraveled - m_previousDistanceTraveled;
        m_previousDistanceTraveled=distanceTraveled;
        
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
        //done check
        if ( (distancePerTick * m_rampDownTicks / 2) > distanceRemaining) {
            if (m_rampDownTicks == 0) {
                m_accelerationState=AccelerationState.DONE;
            } else {
                m_accelerationState=AccelerationState.SLOWING;

            }
        }

        if (m_accelerationState == AccelerationState.ACCELERATING) {
            //accelerating
            System.out.println("accelerating " + m_tickNumber * m_rampUpTicks);

            return (m_targetSpeed * ( m_tickNumber / (double) m_rampUpTicks ) );
        } else if (m_accelerationState == AccelerationState.AT_SPEED) {
            //at_speed
            System.out.println("at speed");


            return m_targetSpeed;
        } else if (m_accelerationState == AccelerationState.SLOWING) {
            //slowing
            m_slowingTick++;
            System.out.println("Slowing "+m_slowingTick + "of" + m_rampDownTicks);
            int ticksRemaining = m_rampDownTicks - m_slowingTick;
            double desiredSpeedPerTick = 0;

            if (ticksRemaining <= 0) {
                return(0.0);
            } else if (ticksRemaining == 1) {
                desiredSpeedPerTick = distanceRemaining;
            } else {
                desiredSpeedPerTick =( distanceRemaining/ ticksRemaining) * 2.3;
            }
            double distancePerPower = m_previousDistanceTraveled / m_calibrationTotalPower;
            
            return (desiredSpeedPerTick / distancePerPower);

        }  else if (m_accelerationState == AccelerationState.DONE) {
            if (m_rampDownTicks > 0){
                return(0.0);
            } else {
                return(m_targetSpeed);
            }
        } else {
            System.out.println("HELP! THIS SHOULD NEVER HAPPEN");
            return(1/0.0);
        }
    }

    @Override
    public boolean isFinished() {
        if (m_accelerationState == AccelerationState.DONE) {
            System.out.println("Drive For Tick finished in " + m_tickNumber + " ticks Deceleration Start Tick " + m_slowingTick + " Ramp Down Ticks " + m_rampDownTicks + " Distance traveled " + m_targetDistance_in);
            return(true);        
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
