package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightStripConstants;

public class Lights extends SubsystemBase {
    
    private final AddressableLED m_LEDstrip = new AddressableLED(LightStripConstants.kLightstripPort);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LightStripConstants.kLightstripLength);

    public Lights() {
        m_LEDstrip.setLength(60);
        
    }

    public void UpdateLights(int state) {
        if (state == LightStripConstants.kOrange) {
            for (var i = 0; i < LightStripConstants.kLightstripLength; i++) {
                m_ledBuffer.setHSV(i, 31, (i+30) ,100);
            }
        } else if (state == LightStripConstants.kYellow) {
            for (var i = 0; i < LightStripConstants.kLightstripLength; i++) {
                m_ledBuffer.setHSV(i, 60, (i+30) ,100);
            }
        } else if (state == LightStripConstants.kPurple) {
            for (var i = 0; i < LightStripConstants.kLightstripLength; i++) {
                m_ledBuffer.setHSV(i, 272, (i+30) ,54);
            }
        }

        m_LEDstrip.setData(m_ledBuffer);
    }
    
}
