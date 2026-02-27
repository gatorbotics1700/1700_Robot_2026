package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDSubsystem;

public class LEDCommands {
  private LEDCommands() {}

  public static Command changeColorCommand(LEDSubsystem ledSubsystem, int r, int g, int b) {
    return new colorCommand(ledSubsystem, r, g, b);
  }

  public static class colorCommand extends Command {
    private final LEDSubsystem ledSubsystem;

    colorCommand(LEDSubsystem ledSubsystem, int r, int g, int b) {
      this.ledSubsystem = ledSubsystem;
      addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
      ledSubsystem.setColor(0, 255, 0); // sets initial LED color to
      System.out.println("initializing");
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
      return true;
    }
  }
}
