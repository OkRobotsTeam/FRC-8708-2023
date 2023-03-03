package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {

    
        PneumaticsControlModule m_pnu = new PneumaticsControlModule();
        DoubleSolenoid m_hook = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        
        
        public HangerSubsystem() { 
            m_hook.set(Value.kReverse);
            
        }

        public void extend() {
            m_hook.set(Value.kForward);

        }

        public void retract() {
            m_hook.set(Value.kReverse);
        }

        public void disable() {
            retract();
        }

      
        

}
