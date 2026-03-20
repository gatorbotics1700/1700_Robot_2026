package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperFloorConstants;
import frc.robot.Constants.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class HopperFloorSubsystem extends SubsystemBase {
  private final TalonFX hopperMotor;
  private double desiredHopperVoltage;

  private DigitalInput dio2 = new DigitalInput(2);
  private DigitalInput dio3 = new DigitalInput(3);
  private DigitalInput dio4 = new DigitalInput(4);
  private DigitalInput dio5 = new DigitalInput(5);
  private DigitalInput dio6 = new DigitalInput(6);
  private DigitalInput dio8 = new DigitalInput(8);

  public HopperFloorSubsystem() {
    hopperMotor = new TalonFX(HopperFloorConstants.HOPPER_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    // TALONFX CONFIGS & MOTION MAGIC VELOCITY VOLTAGE CONTROL // TODO check if this works with
    // motionMagicVelocityVoltage - may want to delete some values
    desiredHopperVoltage = 0.0;
  }

  public void periodic() {
    hopperMotor.setVoltage(desiredHopperVoltage);
    hopperFloorLogs();
  }

  public void setDesiredHopperFloorVoltage(double voltage) {
    this.desiredHopperVoltage = voltage;
  }

  public void hopperFloorLogs() {
    Logger.recordOutput("Mech/Hopper Floor/Desired Voltage", desiredHopperVoltage);
    Logger.recordOutput("Mech/Hopper Floor/Motor Output", hopperMotor.get());

    Logger.recordOutput("DIO/2", dio2.get());
    Logger.recordOutput("DIO/3", dio3.get());
    Logger.recordOutput("DIO/4", dio4.get());
    Logger.recordOutput("DIO/5", dio5.get());
    Logger.recordOutput("DIO/6", dio6.get());
    Logger.recordOutput("DIO/8", dio8.get());
  }
}
