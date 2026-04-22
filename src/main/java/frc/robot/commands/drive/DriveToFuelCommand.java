package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveToFuelConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Calculations;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToFuelCommand extends Command {
  // include any subsystem requirements here
  private final Drive drive;
  private final Vision vision;

  private Pose2d desiredPose;
  private double validTargetSeenTime;
  private Supplier<Pose2d> currentPose;

  // constructor
  public DriveToFuelCommand(Drive drive, Vision vision, Supplier<Pose2d> currentPose) {
    setName("DriveToFuel");
    this.drive = drive;
    this.vision = vision;
    this.currentPose = currentPose;
    validTargetSeenTime = System.currentTimeMillis();
    addRequirements(drive, vision);
  }

  @Override
  public void execute() {
    Pose2d newFuelPose;
    newFuelPose = vision.getFuelPose(currentPose.get());

    if (newFuelPose != null) {
      desiredPose = newFuelPose;
    } else {
      // if we no longer see fuel and it's far enough away, we can safely assume somebody else stole
      // it so we should stop trying to go there
      if (desiredPose != null
          && Calculations.distanceToPoseInMeters(currentPose.get(), desiredPose)
              > DriveToFuelConstants.BLIND_SPOT_DEADBAND) {
        desiredPose = null;
      }
    }

    // determine whether we are at the desired pose
    boolean atDesiredPose = false;
    if (desiredPose != null) {
      double xError = drive.calculateDistanceError(currentPose.get().getX(), desiredPose.getX());
      double yError = drive.calculateDistanceError(currentPose.get().getY(), desiredPose.getY());
      atDesiredPose = xError == 0.0 && yError == 0.0;
    }

    double timeSinceValidTargetSeen = System.currentTimeMillis() - validTargetSeenTime;
    // if we have reached the desiredPose and we have not seen a valid target for a while, spin to
    // try to catch sight of fuel
    if (atDesiredPose
        || desiredPose == null
            && timeSinceValidTargetSeen > DriveToFuelConstants.MAX_IDLE_MILLISECONDS) {
      drive.runVelocity(
          new ChassisSpeeds(0, 0, DriveToFuelConstants.ROTATING_SPEED_RADIANS_PER_SECOND));
    }
    // if we have a desired pose, drive to it
    else if (desiredPose != null) {
      drive.driveToPose(desiredPose);
      validTargetSeenTime = System.currentTimeMillis();
    }
    // otherwise stop
    else {
      drive.stop();
    }
    Logger.recordOutput("DriveToFuel/Desired Pose", desiredPose);
    Logger.recordOutput("DriveToFuel/Current Pose", currentPose.get());
    Logger.recordOutput("DriveToFuel/Time Since Target Seen", timeSinceValidTargetSeen);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
