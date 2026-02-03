package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ShooterSubsystem;

public class ShooterCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private double flywheelSpeed;
  private double kickerSpeed;
  private double startTime;

  public ShooterCommand(ShooterSubsystem shooterSubsystem, double flywheelSpeed) {
    this.shooterSubsystem = shooterSubsystem;
    this.flywheelSpeed = flywheelSpeed;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    shooterSubsystem.setFlywheelSpeed(flywheelSpeed); // shouldn't need to be in periodic
  }

  @Override
  public void execute() {
    if (flywheelSpeed != 0) {
      System.out.println("SHOOTING SHOOTING SHOOTING");
    } else {
      System.out.println("STOPPING");
    }
  }

  // TODO figure out how we want to end this command because I don't think this is it
  @Override
  public boolean isFinished() {
    double timePassed = System.currentTimeMillis() - startTime;
    if (flywheelSpeed == 0) {
      // TODO: can we delete setting it to 0? redundant
      shooterSubsystem.setFlywheelSpeed(0);
      return true;
    }
    if (timePassed > 8000) { // TODO: probably remove this if we want to shoot a lot in a match...
      shooterSubsystem.setFlywheelSpeed(0);
      System.out.println("TIMING OUT");
      return true;
    }
    return false;
  }
}
