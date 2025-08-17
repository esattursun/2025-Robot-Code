package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LEDSubsystem extends SubsystemBase {
    private static final int kPort = 9;
    private static final int kLength = 42;

    public final AddressableLED m_led;
    public final AddressableLEDBuffer m_buffer;

    // private int detectCounter = 0;
    // private int noDetectCounter = 0;

    public LEDSubsystem() {
        m_led = new AddressableLED(kPort);
        m_buffer = new AddressableLEDBuffer(kLength);
        m_led.setLength(kLength);
        m_led.start();
    }

    @Override
    public void periodic() {
                LEDPattern newPattern;

        if(RobotContainer.drivebase.getvalue() == 10){
                        newPattern = LEDPattern.solid(Color.kPurple); 

        }else{
            newPattern = LEDPattern.solid(Color.kDarkBlue); 
        }

        
        newPattern.applyTo(m_buffer);  
        m_led.setData(m_buffer);  
        // m_led.setData(m_buffer);  

    //     // Vision sisteminden etiketleri kontrol et
        // boolean tag1Detected = RobotContainer.drivebase.vision.getleftcam();
    //     boolean tag2Detected = RobotContainer.drivebase.vision.getrightcam();
    //     boolean tag3Detected = RobotContainer.drivebase.vision.getcentercam();

    //     if (tag1Detected || tag2Detected || tag3Detected) {
    //         detectCounter++;  // Etiket tespit edildiyse sayacı artır
    //         noDetectCounter = 0;  // Etiket görüldü, noDetectCounter sıfırla
    //     } else {
    //         noDetectCounter++;  // Etiket bulunmadıysa noDetectCounter'ı artır
    //         detectCounter = 0;  // Etiket kaybolduğunda detectCounter sıfırla
    //     }

    //     LEDPattern newPattern;

    //     // Eğer 5 kere üst üste etiket tespit edildiyse LED yeşil olur
    //     if (detectCounter >= 5) {
    //         newPattern = LEDPattern.solid(Color.kGreen);  // 5 kere gördü, yeşil
    //         noDetectCounter = 0;  // Yeşil olduğunda noDetectCounter sıfırlanır
    //     } 
    //     // Eğer 5 kere üst üste etiket tespit edilmezse LED kırmızı olur
    //     else if (noDetectCounter >= 5) {
    //         newPattern = LEDPattern.solid(Color.kRed);  // 5 kere kayboldu, kırmızı
    //         detectCounter = 0;  // Kırmızı olduğunda detectCounter sıfırlanır
    //         noDetectCounter = 0;  // Kaybolduğunda noDetectCounter sıfırlanır
    //     } else {
    //         return;  // 5 kere görme ya da görmeme durumu gerçekleşmediyse LED değiştirilmez
    //     }

    //     newPattern.applyTo(m_buffer);  // LED pattern uygulama
    //     m_led.setData(m_buffer);  // LED verisini güncelle
     }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_buffer));
    }
}
