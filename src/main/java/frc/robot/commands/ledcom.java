package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class ledcom extends Command {
  // private final LEDSubsystem ld;
  // private int detectCounter = 0;
  // private int noDetectCounter = 0;

  public ledcom(LEDSubsystem ld) {
    // this.ld = ld;
    addRequirements(ld);
  }
 @Override
  public void initialize(){
    
    // noDetectCounter = 0;
    // detectCounter = 0;

  }
  @Override
  public void execute() {
    // // boolean tag1Detected = RobotContainer.drivebase.vision.getleftcam();
    // boolean tag2Detected = RobotContainer.drivebase.vision.getrightcam();
    // // boolean tag3Detected = RobotContainer.drivebase.vision.getcentercam();

    // if (tag1Detected || tag2Detected ) {
    //   detectCounter++; // Görme sayısını artır
    // }   
    
    // if (!tag1Detected && !tag2Detected) {
    //   noDetectCounter++; // Görmeme sayısını artır
    // } else {
    //   noDetectCounter = 0; // Eğer bir kere bile gördüyse sıfırla
    // }

    // LEDPattern newPattern;

    // if (detectCounter >= 5) {
    //   newPattern = LEDPattern.solid(Color.kGreen); // 5 kere toplamda gördüyse yeşil
    //   detectCounter = 0; // Sayaçları sıfırla
    //   noDetectCounter = 0;
    // } else if (noDetectCounter >= 5) {
    //   newPattern = LEDPattern.solid(Color.kRed); // 5 kere üst üste görmediyse kırmızı
    //   detectCounter = 0;
    //   noDetectCounter = 0;
    // } else {
    //   return; // 5 tekrar olmadan LED değiştirme
    // }

    // newPattern.applyTo(ld.m_buffer);
    // ld.m_led.setData(ld.m_buffer);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
