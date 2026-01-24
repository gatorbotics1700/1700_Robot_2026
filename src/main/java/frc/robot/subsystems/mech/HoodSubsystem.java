package frc.robot.subsystems.mech;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class HoodSubsystem extends SubsystemBase {
  public final TalonFX hoodMotor;
  private boolean isTargetting;
  private double hoodVoltage;
  private PIDController pidController;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private static final double kP = 0.0001; // TODO: tune all of these
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double POSITION_DEADBAND_DEGREES = 1.0; // TODO: tune this

  public HoodSubsystem() {
    hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    pidController = new PIDController(kP, kI, kD);
  }

  public void periodic() {
    if (isTargetting == true) {
      turnToPosition(getHoodTargetPosition());
    }
  }

  public void setHoodSpeed(double speed) {
    hoodMotor.setControl(dutyCycleOut.withOutput(speed));
    System.out.println("SETTING HOOD SPEED");
  }

  public double getHoodPosition() {
    return hoodMotor.getPosition().getValueAsDouble()
        * 360
        / Constants.HOOD_GEAR_RATIO; // TODO: check this conversion into degrees
  }

  public double getHoodTargetPosition() {
    return 45; // replace with InterpolatingDoubleTreeMap getter?
  }

  public double getDistanceFromFuelTarget() {
    return 10;
  }

  public void turnToPosition(double targetPositionDegrees) {
    double currentPositionDegrees = getHoodPosition();
    double error = targetPositionDegrees - currentPositionDegrees;

    // Deadband: stop motor when close enough to target
    if (Math.abs(error) < POSITION_DEADBAND_DEGREES) {
      setHoodSpeed(0);
      return;
    }

    // Convert degrees to ticks (motor rotations) for PID
    double targetTicks = targetPositionDegrees * Constants.HOOD_GEAR_RATIO / 360.0;
    double currentTicks = hoodMotor.getPosition().getValueAsDouble();

    hoodVoltage = pidController.calculate(currentTicks, targetTicks);
    setHoodSpeed(hoodVoltage);
  }

  public void setIsTargetting(boolean isTargetting) {
    this.isTargetting = isTargetting;
  }

  public double degreesToTicks(double degrees) {
    return degrees
        / 360
        * Constants.HOOD_SHAFT_REVS_PER_MECH_REV
        * Constants.HOOD_GEAR_RATIO
        * Constants.KRAKEN_TICKS_PER_REV;
  }
}
