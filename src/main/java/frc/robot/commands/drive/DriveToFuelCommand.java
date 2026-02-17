package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class DriveToFuelCommand extends Command {
  // include any subsystem requirements here
  private final Drive drive;
  private final Vision vision;

  // constructor
  public DriveToFuelCommand(Drive drive,Vision vision) {
    this.drive = drive;
    this.vision=vision;
    addRequirements(drive,vision);
  }

  @Override
  public void initialize() {
    // run any methods that should only run once (set desired position, etc)
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Pose2d desiredPose = vision.getFuelPose(currentPose);
    Logger.recordOutput("Odometry/Desired Pose in Intake", desiredPose);
    Logger.recordOutput("Odometry/Current Pose in Intake", currentPose);
    drive.driveToPose(desiredPose);
    System.out.println("this is the desired pose here u go: " + desiredPose);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
