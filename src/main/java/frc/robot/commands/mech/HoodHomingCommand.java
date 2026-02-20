package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.HoodSubsystem;

//TODO: make this start fast, then once it hits the limit switch, go up and then down again slowly to accurately home it

/**
 * Runs the hood toward retract until the retracted limit switch is pressed, then zeros position.
 */
public class HoodHomingCommand extends Command {
  private final HoodSubsystem hoodSubsystem;

  public HoodHomingCommand(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem);
  }

  @Override
  public void initialize() {
    hoodSubsystem.setRetractingToLimitSwitch(true);
  }

  @Override
  public void execute() {
    hoodSubsystem.setHoodVoltage(HoodSubsystem.RETRACT_TO_LIMIT_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.setRetractingToLimitSwitch(false);
  }

  @Override
  public boolean isFinished() {
    if (hoodSubsystem.isRetractedLimitSwitchPressed()) {
      hoodSubsystem.setDesiredAngle(
          hoodSubsystem.getCurrentAngle().plus(new Rotation2d(Math.toRadians(2))));
    }
    return hoodSubsystem.isRetractedLimitSwitchPressed();
  }
}
