package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.mech.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {

  private static final double INTAKING_SPEED =
      9; // TODO get a real number (I just picked my favorite)

  private IntakeCommands() {}

  public static Command RetractIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          Logger.recordOutput("Auto/Intake/Command", "RETRACT");
          Logger.recordOutput("Auto/Intake/TargetAngle", intakeSubsystem.RETRACTED_POSITION);
          intakeSubsystem.setDesiredangle(intakeSubsystem.RETRACTED_POSITION);
        });
  }

  public static Command DeployIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          Logger.recordOutput("Auto/Intake/Command", "DEPLOY");
          Logger.recordOutput("Auto/Intake/TargetAngle", intakeSubsystem.EXTENDED_POSITION);
          intakeSubsystem.setDesiredangle(intakeSubsystem.EXTENDED_POSITION);
        });
  }

  public static Command RunIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          Logger.recordOutput("Auto/Intake/Command", "RUN");
          Logger.recordOutput("Auto/Intake/Speed", INTAKING_SPEED);
          intakeSubsystem.setIntakeSpeed(INTAKING_SPEED);
        });
  }

  public static Command StopIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          Logger.recordOutput("Auto/Intake/Command", "STOP");
          Logger.recordOutput("Auto/Intake/Speed", 0.0);
          intakeSubsystem.setIntakeSpeed(0);
        });
  }

  // TODO: add drive to fuel in this file
}
