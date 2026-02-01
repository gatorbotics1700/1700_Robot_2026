package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.HoodSubsystem;

public class HoodCommand extends Command {
  private HoodSubsystem hoodSubsystem;
  private double startTime;
  private double targetPosition;
  private boolean isTargetting;
  private Translation2d shootingToPosition;

  private static final double kMaxVelocity = 2;
  private static double kMaxAcceleration = 0.5;
  private static double kDt = 0.02;
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final TrapezoidProfile profile = new TrapezoidProfile(m_constraints);

  //   public HoodCommand(HoodSubsystem hoodSubsystem, boolean isTargetting, double targetPosition)
  // {
  //     this.hoodSubsystem = hoodSubsystem;
  //     this.targetPosition = targetPosition;
  //     this.isTargetting = isTargetting;
  //     addRequirements(hoodSubsystem);
  //   }

  public HoodCommand(
      HoodSubsystem hoodSubsystem, boolean isTargetting, Translation2d shootingToPosition) {
    this.hoodSubsystem = hoodSubsystem;
    this.isTargetting = isTargetting;
    this.shootingToPosition = shootingToPosition;
    addRequirements(hoodSubsystem);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    hoodSubsystem.setIsTargetting(isTargetting);
    hoodSubsystem.setShootingToPosition(
        shootingToPosition); // this is for once we start testing targetting
    System.out.println("HOOD STARTING POSITION REVS:" + hoodSubsystem.getHoodPositionMotorRevs());
    System.out.println(
        "HOOD STARTING DEGREES:"
            + hoodSubsystem.motorRevsToDegrees(hoodSubsystem.getHoodPositionMotorRevs()));
    TrapezoidProfile.State currentPositionState =
        new TrapezoidProfile.State(hoodSubsystem.getHoodPositionMotorRevs(), hoodSubsystem.getHoodVoltage());
    TrapezoidProfile.State targetPositionState = new TrapezoidProfile.State(hoodSubsystem.getHoodTargetPosition(shootingToPosition), 0);
    TrapezoidProfile.State setpoint = profile.calculate(kDt, currentPositionState, targetPositionState);
  }

  @Override
  public void execute() {
    // if (!isTargetting) {
    // hoodSubsystem.turnToPosition(hoodSubsystem.degreesToMotorRevs(targetPosition));
    hoodSubsystem.turnToPosition(hoodSubsystem.getHoodTargetPosition(shootingToPosition));
    System.out.println("USING HOOD COMMAND");
    // }
    SmartDashboard.putNumber("Hood target position (degrees)", targetPosition);
  }

  @Override
  public boolean isFinished() {
    if (hoodSubsystem.getHoodVoltage() == 0) {
      return true;
    }
    return false;
  }
}
