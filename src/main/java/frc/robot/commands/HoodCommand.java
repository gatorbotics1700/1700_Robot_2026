package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.HoodSubsystem;

public class HoodCommand extends Command {
  private HoodSubsystem hoodSubsystem;
  private double startTime;
  private double targetPosition;
  private double hoodSpeed;

  public HoodCommand(
      HoodSubsystem hoodSubsystem, boolean isTargetting, double targetPosition, double hoodSpeed) {
    this.hoodSubsystem = hoodSubsystem;
    this.targetPosition = targetPosition;
    this.hoodSpeed = hoodSpeed;
    addRequirements(hoodSubsystem);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    hoodSubsystem.setHoodSpeed(hoodSpeed);
  }

  @Override
  public boolean isFinished() {
    if (hoodSpeed == 0) {
      hoodSubsystem.setHoodSpeed(0);
      return true;
    }
    return false;
  }
}
