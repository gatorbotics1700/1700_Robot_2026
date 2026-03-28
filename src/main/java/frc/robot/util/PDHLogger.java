package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.Logger;

public class PDHLogger {
  private final PowerDistribution pdh;

  private static final int NUM_CHANNELS = 24;

  public PDHLogger() {
    pdh = new PowerDistribution(1, ModuleType.kRev);
  }

  public PDHLogger(int canId) {
    pdh = new PowerDistribution(canId, ModuleType.kRev);
  }

  public void log() {

    double[] channelCurrents = new double[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
      channelCurrents[i] = pdh.getCurrent(i);
    }
    
    Logger.recordOutput("Power/PDH/ChannelCurrents", channelCurrents);

    // Log total current
    Logger.recordOutput("Power/PDH/TotalCurrent", pdh.getTotalCurrent());

    // Log voltage
    Logger.recordOutput("Power/PDH/Voltage", pdh.getVoltage());

    // Log total power
    Logger.recordOutput("Power/PDH/TotalPower", pdh.getTotalPower());

    // Log total energy
    Logger.recordOutput("Power/PDH/TotalEnergy", pdh.getTotalEnergy());

    // Log temperature
    Logger.recordOutput("Power/PDH/Temperature", pdh.getTemperature());

    //Log all currents 
    Logger.recordOutput("Power/PDH/Temperature", pdh.getAllCurrents());
  }
}
