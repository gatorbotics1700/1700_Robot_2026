package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperFloorSubsystem extends SubsystemBase {
  public static final double HOPPER_FLOOR_SPEED = 0;
  public final TalonFX hopperMotor;
  private double hopperVelocity;
  private static TalonFXConfiguration talonFXConfigs;
  private static Slot0Configs slot0Configs;
  private static MotionMagicVelocityVoltage m_velocity;

  public HopperFloorSubsystem() {
    hopperMotor = new TalonFX(Constants.HOPPER_MOTOR_CAN_ID, ""); // TunerConstants.mechCANBus);

    // TALONFX CONFIGS & MOTION MAGIC VELOCITY VOLTAGE CONTROL // TODO check if this works with
    // motionMagicVelocityVoltage - may want to delete some values
    m_velocity = new MotionMagicVelocityVoltage(0);
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)); // TODO check invert

    slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kG =
        0.2128; // Add 0.2128 V output to overcome gravity (tuned in early feedforward testing)
    slot0Configs.kS =
        0.25; // Add 0.01 V output to overcome static friction (just a guesstimate, but this might
    // just be 0
    slot0Configs.kV = 0.16; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12V output\
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // a velocity error of 1 rps results in 0.1 V output

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.16; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    hopperMotor.getConfigurator().apply(talonFXConfigs, 0.050);

    m_velocity.Slot = 0;
  }

  public void periodic() {
    hopperMotor.setControl(m_velocity.withVelocity(hopperVelocity));
  }

  public void setHopperFloorVelocity(double hopperVelocity) {
    this.hopperVelocity = hopperVelocity;
  }
}
