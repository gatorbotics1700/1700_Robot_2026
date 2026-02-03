package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.HopperFloorSubsystem;

public class TransitionCommand extends Command {

  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final double hopperSpeed;
  private double startTime;

  public TransitionCommand(HopperFloorSubsystem hopperFloorSubsystem, double hopperSpeed) {
    this.hopperFloorSubsystem = hopperFloorSubsystem;
    this.hopperSpeed = hopperSpeed;
    addRequirements(hopperFloorSubsystem);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    hopperFloorSubsystem.setHopperFloorSpeed(hopperSpeed);
  }

  @Override
  public void execute() {
    // shouldn't need to do anything because once we set speed in the init the subsystem should just
    // keep running it at that speed!
  }

  // TODO figure out how we want to end this command
  @Override
  public boolean isFinished() {
    // if (hopperVoltage == 0) {
    //   return true;
    // } else
    if (System.currentTimeMillis() - startTime > 10000) {
      hopperFloorSubsystem.setHopperFloorSpeed(0);
      System.out.println("TIMING OUT");
      return true;
    }
    return false;
  }
}
