package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  public final TalonFX hoodMotor;
  private boolean isTargetting;
  private double hoodVoltage;
  private Translation2d shootingToPosition;
  private double pidOutput;
  private double ffOutput;

  TrapezoidProfile.State currentPositionState;
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final TrapezoidProfile profile = new TrapezoidProfile(m_constraints);
  private ProfiledPIDController pidController =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  private static final double kDt = 0.02; // time step
  private static final double kMaxVelocity = 2;
  private static double kMaxAcceleration = 0.5;
  private static final double kP = 0.0; // TODO: tune all of these
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.0;
  private static final double kG = 0.2128; // Tuned, do not change
  private static final double kV = 0.5;
  private static final double kA = 0;
  private final double POSITION_DEADBAND_REVS = degreesToMotorRevs(1); // TODO: tune
  private static double currentPositionRevs;
  private static double targetPositionRevs;
  private TrapezoidProfile.State setpoint;

  private Supplier<Pose2d> robotPose;

  public HoodSubsystem(Supplier<Pose2d> robotPose) {
    hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    hoodMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    this.robotPose = robotPose;
  }

  @Override
  public void periodic() {
    if (isTargetting) {
      turnToPosition(getHoodTargetPosition(shootingToPosition));
    }
    currentPositionRevs = getHoodPositionMotorRevs();
    SmartDashboard.putNumber("Hood current position (revs)", currentPositionRevs);
    SmartDashboard.putNumber(
        "Hood current position (degrees)", motorRevsToDegrees(currentPositionRevs));

    Logger.recordOutput("Robot/pidOutput", pidOutput);
    Logger.recordOutput("Robot/feedforwardOutput", ffOutput);
    Logger.recordOutput("Robot/velocity", hoodMotor.getVelocity().getValueAsDouble());
  }

  public void setShootingToPosition(
      Translation2d shootingToPosition) { // this is for once we start testing targetting
    this.shootingToPosition = shootingToPosition;
  }

  public void setHoodVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
    System.out.println("SETTING HOOD SPEED: " + voltage);
  }

  public double getHoodVoltage() {
    return hoodVoltage;
  }

  public double getHoodPositionMotorRevs() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public double getHoodTargetPosition(Translation2d shootingToPosition) {
    Pose2d currentRobotPose = robotPose.get();
    double deltaY = Math.abs(shootingToPosition.getY() - currentRobotPose.getY());
    double deltaX = Math.abs(shootingToPosition.getX() - currentRobotPose.getX());
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    double degreesPosition =
        distance
            / 2; // replace with InterpolatingDoubleTreeMap getter to find degrees using distance
    System.out.println("Distance from position shooting at:" + distance);
    System.out.println("Hood desired position degrees:" + degreesPosition);
    return degreesToMotorRevs(degreesPosition);
  }

  public double getDistanceFromFuelTarget() {
    return 10;
  }

  public void turnToPosition(double targetPositionRevs) {
    System.out.println("TARGET HOOD POSITION DEGREES: " + motorRevsToDegrees(targetPositionRevs));
    SmartDashboard.putNumber(
        "Hood target position (degrees)",
        motorRevsToDegrees(targetPositionRevs)); // things only show up in elastic when in periodic
    System.out.println("CURRENT HOOD POS DEGREES: " + motorRevsToDegrees(currentPositionRevs));
    double errorRevs = targetPositionRevs - currentPositionRevs;
    System.out.println("HOOD POSITION ERROR DEGREES: " + motorRevsToDegrees(errorRevs));

    // Deadband: stop motor when close enough to target
    if (Math.abs(errorRevs) < POSITION_DEADBAND_REVS) {
      hoodVoltage = 0;
      System.out.println("HOOD AT POSITION. STOPPING");
    } else {
      hoodVoltage = calculateVoltage(targetPositionRevs);
      System.out.println("MOVING HOOD!!");
    }
    setHoodVoltage(hoodVoltage);
  }

  public void setTrapezoidProfile(double targetPositionRevs){
    this.targetPositionRevs = targetPositionRevs;
  }

  private double calculateVoltage(double targetPositionRevs) {
    pidController.setGoal(targetPositionRevs);
    pidOutput = pidController.calculate(currentPositionRevs, targetPositionRevs);
    System.out.println("CALCULATED PID OUTPUT:" + pidOutput);
    ffOutput =
        feedforward.calculate(
            (Math.PI / 180) * motorRevsToDegrees(setpoint.position),
            setpoint.velocity); // (Math.PI / 180) * motorRevsToDegrees(targetPositionRevs)
    System.out.println(
        "DESIRED POSITION IN RADIANS:" + (Math.PI / 180) * motorRevsToDegrees(targetPositionRevs));
    System.out.println(
        "CURRENT POSITION STATE PROFILE:"
            + currentPositionState.position
            + "VELOCITY:"
            + currentPositionState.velocity);
    System.out.println(
        "CALCULATED POSITION STATE PROFILE:" + setpoint.position + "VELOCITY:" + setpoint.velocity);
    System.out.println("CALCULATED FEEDFOWARD OUTPUT" + ffOutput);
    double voltage = pidOutput + ffOutput;
    return voltage;
  }

  public void setIsTargetting(boolean isTargetting) {
    this.isTargetting = isTargetting;
  }

  public double degreesToMotorRevs(double degrees) {
    return degrees / 360 * Constants.HOOD_SHAFT_REVS_PER_MECH_REV * Constants.HOOD_GEAR_RATIO;
  }

  public double motorRevsToDegrees(double revs) {
    return revs / Constants.HOOD_GEAR_RATIO / Constants.HOOD_SHAFT_REVS_PER_MECH_REV * 360;
  }
}
