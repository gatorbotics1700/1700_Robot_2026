package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HoodSubsystem extends SubsystemBase {
  public static final Rotation2d RETRACTED_POSITION =
      new Rotation2d(Math.toRadians(20)); // TODO find a real number for this
  public final TalonFX hoodMotor;
  // some motion magic stuff here
  private TalonFXConfiguration talonFXConfigs;
  private Rotation2d desiredAngle;
  private final double POSITION_DEADBAND_DEGREES = 0.5; // TODO: tune
  private final int HOOD_GEARBOX_RATIO = 9; // TODO find the real value
  private final int HOOD_SHAFT_REVS_PER_MECH_REV = 155 / 15; // TODO find real value
  private static double currentPositionTicks;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  final MotionMagicExpoVoltage m_request;

  public HoodSubsystem() {
    hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    desiredAngle = new Rotation2d(0);

    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    // TODO: make tuneable constants
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    LoggedNetworkNumber tunekG =
        new LoggedNetworkNumber(
            "/Tuning/kG",
            0.2128); // Add 0.2128 V output to overcome gravity (tuned in early feedforward
    // testing)

    LoggedNetworkNumber tunekS =
        new LoggedNetworkNumber(
            "/Tuning/kS",
            0.25); // 0.01; // Add 0.01 V output to overcome static friction (just a guesstimate,
    // but
    // this value
    // might just be 0)
    LoggedNetworkNumber tunekV = new LoggedNetworkNumber("/Tuning/kV", 0.16);

    // A velocity target of 1 rps results in 0.12 V output
    LoggedNetworkNumber tunekA = new LoggedNetworkNumber("/Tuning/kA", 0.01);

    // An acceleration of 1 rps/s requires 0.01 V output
    LoggedNetworkNumber tunekP = new LoggedNetworkNumber("/Tuning/kP", 4.8);

    // A position error of 2.5 rotations results in 12 V output
    LoggedNetworkNumber tunekI = new LoggedNetworkNumber("/Tuning/kI", 0);

    // no output for integrated error
    LoggedNetworkNumber tunekD = new LoggedNetworkNumber("/Tuning/kD", 0.1);
    // A velocity error of 1 rps results in 0.1 V output

    slot0Configs.kG = tunekG.get();
    slot0Configs.kS = tunekS.get();
    slot0Configs.kV = tunekV.get();
    slot0Configs.kA = tunekA.get();
    slot0Configs.kP = tunekP.get();
    slot0Configs.kI = tunekI.get();
    slot0Configs.kD = tunekD.get();

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
    // motionMagicConfigs.MotionMagicExpo_kV = 0.16; // kV is around 0.12 V/rps
    // motionMagicConfigs.MotionMagicExpo_kA = 0.1; // 0.015; // Use a slower kA of 0.1 V/(rps/s)

    LoggedNetworkNumber tuneCruiseVelocity = new LoggedNetworkNumber("/Tuning/CruiseVelocity", 0);
    LoggedNetworkNumber tuneExpo_kV = new LoggedNetworkNumber("/Tuning/Expo_kV", 0.16);
    LoggedNetworkNumber tuneExpo_kA = new LoggedNetworkNumber("/Tuning/Expo_kA", 0.1);

    motionMagicConfigs.MotionMagicCruiseVelocity = tuneCruiseVelocity.get();
    motionMagicConfigs.MotionMagicExpo_kV = tuneExpo_kV.get();
    motionMagicConfigs.MotionMagicExpo_kA = tuneExpo_kA.get();

    hoodMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void periodic() {
    // I used a fake pid as a placeholeder, but we should turn to position using motion magic
    // double angleError = getCurrentAngle().getDegrees() - desiredAngle.getDegrees();
    // if (Math.abs(angleError) > POSITION_DEADBAND_DEGREES) {
    configMotor();
    hoodMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));

    Logger.recordOutput("hood desired angle", desiredAngle.getDegrees());
    Logger.recordOutput("hood motor output", hoodMotor.get());
    Logger.recordOutput("hood current angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("hood current velocity", hoodMotor.getVelocity().getValueAsDouble());
  }

  public void setDesiredAngle(
      Rotation2d desiredAngle) { // this is for once we start testing targetting
    this.desiredAngle = desiredAngle;
  }

  public void setHoodSpeed(double speed) {
    hoodMotor.setControl(dutyCycleOut.withOutput(speed));
    System.out.println("SETTING HOOD SPEED: " + speed);
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = hoodMotor.getPosition().getValueAsDouble();
    double hoodAngleDegrees =
        motorPositionRevs / HOOD_GEARBOX_RATIO / HOOD_SHAFT_REVS_PER_MECH_REV * 360 % 360;
    return new Rotation2d(
        Math.toRadians(
            hoodAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }

  public double degreesToRevs(double hoodAngleDegrees) {
    return hoodAngleDegrees / 360.0 * HOOD_SHAFT_REVS_PER_MECH_REV * HOOD_GEARBOX_RATIO;
  }

  public void setHoodVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

  private void configMotor() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    LoggedNetworkNumber tunekG =
        new LoggedNetworkNumber(
            "/Tuning/kG",
            0.2128); // Add 0.2128 V output to overcome gravity (tuned in early feedforward
    // testing)

    LoggedNetworkNumber tunekS =
        new LoggedNetworkNumber(
            "/Tuning/kS",
            0.25); // 0.01; // Add 0.01 V output to overcome static friction (just a guesstimate,
    // but
    // this value
    // might just be 0)
    LoggedNetworkNumber tunekV = new LoggedNetworkNumber("/Tuning/kV", 0.16);

    // A velocity target of 1 rps results in 0.12 V output
    LoggedNetworkNumber tunekA = new LoggedNetworkNumber("/Tuning/kA", 0.01);

    // An acceleration of 1 rps/s requires 0.01 V output
    LoggedNetworkNumber tunekP = new LoggedNetworkNumber("/Tuning/kP", 4.8);

    // A position error of 2.5 rotations results in 12 V output
    LoggedNetworkNumber tunekI = new LoggedNetworkNumber("/Tuning/kI", 0);

    // no output for integrated error
    LoggedNetworkNumber tunekD = new LoggedNetworkNumber("/Tuning/kD", 0.1);
    // A velocity error of 1 rps results in 0.1 V output

    slot0Configs.kG = tunekG.get();
    slot0Configs.kS = tunekS.get();
    slot0Configs.kV = tunekV.get();
    slot0Configs.kA = tunekA.get();
    slot0Configs.kP = tunekP.get();
    slot0Configs.kI = tunekI.get();
    slot0Configs.kD = tunekD.get();

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
    // motionMagicConfigs.MotionMagicExpo_kV = 0.16; // kV is around 0.12 V/rps
    // motionMagicConfigs.MotionMagicExpo_kA = 0.1; // 0.015; // Use a slower kA of 0.1 V/(rps/s)

    LoggedNetworkNumber tuneCruiseVelocity = new LoggedNetworkNumber("/Tuning/CruiseVelocity", 0);
    LoggedNetworkNumber tuneExpo_kV = new LoggedNetworkNumber("/Tuning/Expo_kV", 0.16);
    LoggedNetworkNumber tuneExpo_kA = new LoggedNetworkNumber("/Tuning/Expo_kA", 0.1);

    motionMagicConfigs.MotionMagicCruiseVelocity = tuneCruiseVelocity.get();
    motionMagicConfigs.MotionMagicExpo_kV = tuneExpo_kV.get();
    motionMagicConfigs.MotionMagicExpo_kA = tuneExpo_kA.get();

    if (talonFXConfigs != this.talonFXConfigs) {
      hoodMotor.getConfigurator().apply(talonFXConfigs);
      this.talonFXConfigs = talonFXConfigs;
    }
  }
}
