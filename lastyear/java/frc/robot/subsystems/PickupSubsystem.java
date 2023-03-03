package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class PickupSubsystem extends SubsystemBase {

    
        PneumaticsControlModule m_pnu = new PneumaticsControlModule();
        DoubleSolenoid m_arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        CANSparkMax m_motor = new CANSparkMax(Constants.kPickupMotor ,MotorType.kBrushed);
        
        public PickupSubsystem() { 

            m_arm.set(Value.kReverse);
            
        }

        public void armDown() {
            System.out.println("armDown");
            m_arm.set(Value.kForward);

        }

        public void armUp() {
            System.out.println("armUp");

            m_arm.set(Value.kReverse);
        }

        public void runMotor() {
            System.out.println("runMotor");

            m_motor.set(1);
        }

        public void stopMotor() {
            System.out.println("stopMotor");

            m_motor.set(0);
        }

        public void reverse() {
            m_motor.set(-1);
        }

        public void run() {
            runMotor();
        }

        public void stop() {
            stopMotor();
        }

        public void pickupDown() {
            armDown();
        }

        public void pickupUp() {
            armUp();
        }

}
