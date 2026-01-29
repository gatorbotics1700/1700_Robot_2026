package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/* code is suuposed to make an automated command for the robot
to go over the bump as one wheel - two wheel - one wheel */

public class DriveOverBumpCommand extends Command {

  private static final double ANGLE_KP = 5; // TODO: (ak does not know)
  private static final double ANGLE_KD = 0.4; // TODO: (ak does not know)
  private static final double ANGLE_MAX_VELOCITY = 5; // TODO: (ak does not know)
  private static final double ANGLE_MAX_ACCELERATION = 5; // TODO: (ak does not know)

  private final Drive drive;
  private final double distanceMeters;
  private final double speedMetersPerSec;
  private final double approachAngleDegrees; // angle to approach bump

  private final ProfiledPIDController headingController;

  private Pose2d startPose;
  private Rotation2d targetHeading;
  private boolean doneRotating;

  // Default values
  private static final double DEFAULT_SPEED_OVER_BUMP =
      1.0; // slow for bump crossing (i don't want to catch air)
  private static final double DEFAULT_DISTANCE_OVER_BUMP = 1.0; // Distance over bump
  private static final double DEFAULT_APPROACH_ANGLE_BUMP =
      30.0; // 30 degrees off straight (idk if this is right)

  public static DriveOverBumpCommand create(Drive drive) {
    return new DriveOverBumpCommand(
        drive, DEFAULT_DISTANCE_OVER_BUMP, DEFAULT_SPEED_OVER_BUMP, DEFAULT_APPROACH_ANGLE_BUMP);
  }

  public static DriveOverBumpCommand create(
      Drive drive, double distanceMeters, double speedMetersPerSec, double approachAngleDegrees) {
    return new DriveOverBumpCommand(drive, distanceMeters, speedMetersPerSec, approachAngleDegrees);
  }

  private DriveOverBumpCommand(
      Drive drive, double distanceMeters, double speedMetersPerSec, double approachAngleDegrees) {
    this.drive = drive;
    this.distanceMeters = distanceMeters;
    this.speedMetersPerSec = Math.abs(speedMetersPerSec);
    this.approachAngleDegrees = approachAngleDegrees;

    this.headingController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startPose = drive.getPose();

    // target heading is current heading + approach angle
    targetHeading = drive.getRotation().plus(Rotation2d.fromDegrees(approachAngleDegrees));
    headingController.reset(drive.getRotation().getRadians());
    doneRotating = false;
  }

  @Override
  public void execute() {
    double headingError = targetHeading.getRadians() - drive.getRotation().getRadians();
    double headingCorrection =
        headingController.calculate(drive.getRotation().getRadians(), targetHeading.getRadians());

    // check if we're close enough to target angle
    if (!doneRotating && Math.abs(Math.toDegrees(headingError)) < 2.0) {
      doneRotating = true;
      startPose = drive.getPose(); // reset start pose for distance tracking
    }

    double forwardSpeed = 0;
    if (doneRotating) {
      double direction = distanceMeters >= 0 ? 1.0 : -1.0;
      forwardSpeed = direction * speedMetersPerSec;
    }

    drive.runVelocity(new ChassisSpeeds(forwardSpeed, 0, headingCorrection));

    
    double distanceTraveled =
        drive.getPose().getTranslation().getDistance(startPose.getTranslation());
    Logger.recordOutput("DriveOverBump/DoneRotating", doneRotating);
    Logger.recordOutput("DriveOverBump/DistanceTraveled", distanceTraveled);
    Logger.recordOutput("DriveOverBump/TargetDistance", Math.abs(distanceMeters));
    Logger.recordOutput("DriveOverBump/HeadingErrorDeg", Math.toDegrees(headingError));
    Logger.recordOutput("DriveOverBump/ForwardSpeed", forwardSpeed);


        // Logging

  }

  @Override
  public boolean isFinished() {
    if (!doneRotating) {
      return false;
    }
    double distanceTraveled =
        drive.getPose().getTranslation().getDistance(startPose.getTranslation());
    return distanceTraveled >= Math.abs(distanceMeters);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
