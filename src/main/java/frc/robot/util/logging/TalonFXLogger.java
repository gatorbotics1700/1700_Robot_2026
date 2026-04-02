package frc.robot.util.logging;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

/**
 * AdvantageKit logging for Phoenix 6 {@link TalonFX} telemetry.
 *
 * <p>Use a stable prefix per motor, e.g. {@code "Mech/Turret/motor"} or {@code
 * "Mech/Shooter/leftFlywheel"}.
 *
 * <p>Mechanism-specific quantities (e.g. angle in mechanism units for SysId) may still be logged
 * from the subsystem alongside these motor signals.
 */
public final class TalonFXLogger {

  private TalonFXLogger() {}

  public static void log(TalonFX motor, String category, String mechanismName) {
    log(motor, category, mechanismName, "");
  }

  public static void configureTelemetryUpdateHz(TalonFX motor) {
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motor.getVelocity(),
        motor.getStatorCurrent(),
        motor.getMotorVoltage(),
        motor.getPosition(),
        motor.getSupplyVoltage(),
        motor.getDeviceTemp(),
        motor.getClosedLoopReference(),
        motor.getClosedLoopError(),
        motor.getClosedLoopFeedForward());
  }

  public static void log(TalonFX motor, String category, String mechanismName, String motorName) {
    String prefix = category + "/" + mechanismName + (motorName.equals("") ? "" : "/" + motorName);

    var velocity = motor.getVelocity();
    var statorCurrent = motor.getStatorCurrent();
    var motorVoltage = motor.getMotorVoltage();
    var position = motor.getPosition();
    var supplyVoltage = motor.getSupplyVoltage();
    var deviceTemp = motor.getDeviceTemp();
    var closedLoopRef = motor.getClosedLoopReference();
    var closedLoopErr = motor.getClosedLoopError();
    var closedLoopFf = motor.getClosedLoopFeedForward();

    BaseStatusSignal.refreshAll(
        velocity,
        statorCurrent,
        motorVoltage,
        position,
        supplyVoltage,
        deviceTemp,
        closedLoopRef,
        closedLoopErr,
        closedLoopFf);

    Logger.recordOutput(prefix + "/Motor Output", motor.get());
    // NOTE: For SysID, velocity needs to be in proper units that rely on gear ratios, so will need
    // to log in the subsystem as well
    Logger.recordOutput(prefix + "/Motor Velocity", velocity.getValueAsDouble());
    Logger.recordOutput(prefix + "/StatorCurrent", statorCurrent.getValueAsDouble());
    Logger.recordOutput(
        category + "/All Stator Currents/" + (motorName.equals("") ? mechanismName : motorName),
        statorCurrent.getValueAsDouble());
    Logger.recordOutput(prefix + "/MotorVoltage", motorVoltage.getValueAsDouble());
    // NOTE: position needs to be in mechanism units not motor units, so log in subsystem as well
    Logger.recordOutput(prefix + "/Motor Position", position.getValueAsDouble());
    Logger.recordOutput(prefix + "/SupplyVoltage", supplyVoltage.getValueAsDouble());
    Logger.recordOutput(prefix + "/DeviceTemp", deviceTemp.getValueAsDouble());

    Logger.recordOutput(
        prefix + "/ClosedLoop/ClosedLoopReference", closedLoopRef.getValueAsDouble());
    Logger.recordOutput(prefix + "/ClosedLoop/ClosedLoopError", closedLoopErr.getValueAsDouble());
    Logger.recordOutput(
        prefix + "/ClosedLoop/ClosedLoopFeedForward", closedLoopFf.getValueAsDouble());
  }
}
