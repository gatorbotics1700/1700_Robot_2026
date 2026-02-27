package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

  private final CANdle candle;
  private int r, g, b;

  public LEDSubsystem() {
    this.candle = new CANdle(LEDConstants.CANdle_CAN_ID);

    StripTypeValue strip_type = StripTypeValue.RGB; // setting type of LED strip
    LEDConfigs ledConfig =
        new LEDConfigs()
            .withBrightnessScalar(0.5)
            .withStripType(strip_type); // configures led brightness
    CANdleConfiguration config = new CANdleConfiguration().withLED(ledConfig);
    candle.getConfigurator().apply(config);
  }

  public void setColor(int new_r, int new_g, int new_b) {
    this.r = new_r;
    this.g = new_g;
    this.b = new_b;
    SolidColor solidColor = new SolidColor(8, 67);
    candle.setControl(solidColor.withColor(new RGBWColor(r, g, b, 0)));
  }
}
