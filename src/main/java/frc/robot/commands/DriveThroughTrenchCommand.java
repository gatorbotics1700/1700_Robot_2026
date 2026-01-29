package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveThroughTrenchCommand extends Command {
  private static final double ANGLE_KP = 5.0; // ak does not know
  private static final double ANGLE_KD = 0.5; // like idk
  private static final double ANGLE_MAX_VELOCITY = 10; // like idk
  private static final double ANGLE_MAX_ACCELERATION = 10; // likd idk

  private final Drive drive;
  private final double distanceMeters;
  private final double speedMetersPerSec;

  private final ProfiledPIDController headingController;
  private Pose2d startPose;
  private Rotation2d targetHeading;

  private static final double DEFAULT_SPEED_DA_TRENCH = 1.5;
  private static final double DEFAULT_DISTANCE_DA_TRENCH =
      2.0; // positive = forward, negative = backward

  public static DriveThroughTrenchCommand create(Drive drive) {
    return new DriveThroughTrenchCommand(
        drive, DEFAULT_DISTANCE_DA_TRENCH, DEFAULT_SPEED_DA_TRENCH);
  }

  private DriveThroughTrenchCommand(Drive drive, double distanceMeters, double speedMetersPerSec) {
    this.drive = drive;
    this.distanceMeters = distanceMeters;
    this.speedMetersPerSec = Math.abs(speedMetersPerSec);

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
    // capture starting pose and heading
    startPose = drive.getPose();
    targetHeading = new Rotation2d(0); // drive.getRotation(); do we want to just stay at what it is
    headingController.reset(drive.getRotation().getRadians());
    Logger.recordOutput("DriveThroughTrench/CommandStarted", true);
  }

  @Override
  public void execute() {
    // calc distance traveled from start
    double distanceTraveled =
        drive.getPose().getTranslation().getDistance(startPose.getTranslation());

    // calc heading correction to stay straight
    double headingCorrection =
        headingController.calculate(drive.getRotation().getRadians(), targetHeading.getRadians());

    // determine drive direction based on sign - pos + & neg - TODO: talk through logic code review
    double direction;
    if (distanceMeters > 0) {
      direction = 1.0;
    } else if (distanceMeters < 0) {
      direction = -1.0;
    } else {
      direction = 0.0;
    }

    // drive robot-relative (straight forward/backward)
    ChassisSpeeds speeds = new ChassisSpeeds(direction * speedMetersPerSec, 0.0, headingCorrection);
    drive.runVelocity(speeds);

    Logger.recordOutput("DriveThroughTrench/DistanceTraveled", distanceTraveled);
    Logger.recordOutput("DriveThroughTrench/TargetDistance", Math.abs(distanceMeters));
    Logger.recordOutput(
        "DriveThroughTrench/HeadingError",
        Math.toDegrees(drive.getRotation().getRadians() - targetHeading.getRadians()));
  }

  @Override
  public boolean isFinished() {
    // stop when traveled the target distance
    double distanceTraveled =
        drive.getPose().getTranslation().getDistance(startPose.getTranslation());
    return distanceTraveled >= Math.abs(distanceMeters);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
