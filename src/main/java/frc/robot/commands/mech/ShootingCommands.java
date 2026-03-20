package frc.robot.commands.mech;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.Constants.HopperFloorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.util.Calculations;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotParameters;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootingCommands {
  public ShootingCommands() {}

  // Current logic is that if the flywheel speed is 0 then we're just tracking and
  // if the flywheel
  // speed is not zero then we're trying to shoot, but we may decide we want a
  // separate command
  // for
  // just tracking

  public static class ShootingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private Supplier<Pose2d> drivetrainPose;
    private final HopperFloorSubsystem hopperFloorSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final Supplier<ShotParameters> shotParameters;

    // private final TurretSubsystem turretSubsystem;

    public ShootingCommand(
        ShooterSubsystem shooterSubsystem,
        HoodSubsystem hoodSubsystem,
        HopperFloorSubsystem hopperFloorSubsystem,
        // TurretSubsystem turretSubsystem,
        Supplier<Pose2d> drivetrainPose,
        Supplier<ShotParameters> shotParameters) {
      this.shooterSubsystem = shooterSubsystem;
      this.hopperFloorSubsystem = hopperFloorSubsystem;
      this.drivetrainPose = drivetrainPose;
      this.hoodSubsystem = hoodSubsystem;
      this.shotParameters = shotParameters;
      // this.turretSubsystem = turretSubsystem;
      addRequirements(shooterSubsystem, hoodSubsystem, hopperFloorSubsystem);
      setName("ShootingCommand");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      // double desiredRotorVelocity =
      //     ShooterSubsystem.launchSpeedToRotorSpeed(validStationaryShot.shotSpeed);
      shooterSubsystem.setDesiredRotorVelocity(
          shotParameters.get().shotSpeed); // set velocity to our desired velocity
      hopperFloorSubsystem.setDesiredHopperFloorVoltage(HopperFloorConstants.HOPPER_FLOOR_VOLTAGE);
      if (Math.abs(shooterSubsystem.getFlywheelRotorVelocity() - shotParameters.get().shotSpeed)
          < ShooterConstants
              .FLYWHEEL_SPEED_DEADBAND) { // once flywheel is running close to our desired velocity
        // turretSubsystem.setDesiredAngle(shotParameters.turretAngle);
        hoodSubsystem.setDesiredAngle(
            hoodSubsystem.convertLaunchAngleToHoodAngle(shotParameters.get().hoodAngle));
        shooterSubsystem.setDesiredTransitionVoltage(ShooterConstants.TRANSITION_VOLTAGE);
      }
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public void end(boolean interrupted) {
      hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
      shooterSubsystem.setDesiredTransitionVoltage(0);
    }
  }

  public static class StopShooting extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final HopperFloorSubsystem hopperFloorSubsystem;

    public StopShooting(
        ShooterSubsystem shooterSubsystem, HopperFloorSubsystem hopperFloorSubsystem) {
      this.shooterSubsystem = shooterSubsystem;
      this.hopperFloorSubsystem = hopperFloorSubsystem;
      addRequirements(shooterSubsystem, hopperFloorSubsystem);
      setName("StopShooting");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      if (shooterSubsystem.getFlywheelRotorVelocity() != 0) {
        shooterSubsystem.setDesiredRotorVelocity(0);
        shooterSubsystem.setDesiredTransitionVoltage(0);
        hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
      } else {
        System.out.println("SHOOTER ALREADY STOPPED");
      }
    }

    @Override
    public boolean isFinished() {
      if (shooterSubsystem.getFlywheelRotorVelocity() == 0) {
        return true;
      }
      return false;
    }
  }

  public static Command StationaryShootingCommand(
      ShooterSubsystem shooterSubsystem,
      HoodSubsystem hoodSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> drivetrainPose) {
    Supplier<ShotParameters> closestShotParameters = null;
    Logger.recordOutput("Mech/Shooter/Stationary/RED_RIGHT", ShooterConstants.RED_SHOT.pose);
    Logger.recordOutput("Mech/Shooter/Stationary/BLUE_LEFT", ShooterConstants.BLUE_SHOT.pose);
    Logger.recordOutput(
        "Mech/Shooter/Stationary/RED_RIGHT distance",
        Calculations.distanceToPoseInMeters(drivetrainPose.get(), ShooterConstants.RED_RIGHT.pose));
    Logger.recordOutput(
        "Mech/Shooter/Stationary/BLUE_LEFT distance",
        Calculations.distanceToPoseInMeters(drivetrainPose.get(), ShooterConstants.BLUE_LEFT.pose));
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      for (ShotParameters shot : ShooterConstants.STATIONARY_BLUE_SHOTS_ARRAY) {
        if (closestShotParameters == null
            || Calculations.distanceToPoseInMeters(drivetrainPose.get(), shot.pose)
                < Calculations.distanceToPoseInMeters(
                    drivetrainPose.get(), closestShotParameters.get().pose)) {
          closestShotParameters =
              () -> {
                return shot;
              };
        }
      }
    } else {
      for (ShotParameters shot : ShooterConstants.STATIONARY_RED_SHOTS_ARRAY) {
        if (closestShotParameters == null
            || Calculations.distanceToPoseInMeters(drivetrainPose.get(), shot.pose)
                < Calculations.distanceToPoseInMeters(
                    drivetrainPose.get(), closestShotParameters.get().pose)) {
          closestShotParameters =
              () -> {
                return shot;
              };
        }
      }
    }

    Logger.recordOutput(
        "Mech/ShootingCommand/Closest shot parameter", closestShotParameters.get().pose);

    return AutoBuilder.pathfindToPose(
            closestShotParameters.get().pose,
            new PathConstraints(4, 12, Math.toRadians(700), Math.toRadians(1000)))
        .andThen(
            new ShootingCommand(
                shooterSubsystem,
                hoodSubsystem,
                hopperFloorSubsystem,
                drivetrainPose,
                closestShotParameters))
        .withName("StationaryShootingCommand");
  }

  public static class ShootOnTheMoveCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final HopperFloorSubsystem hopperFloorSubsystem;
    private final TurretSubsystem turretSubsystem;
    private Supplier<Pose2d> drivetrainPose;
    private Supplier<ChassisSpeeds> drivetrainVelocity;

    public ShootOnTheMoveCommand(
        ShooterSubsystem shooterSubsystem,
        HoodSubsystem hoodSubsystem,
        HopperFloorSubsystem hopperFloorSubsystem,
        TurretSubsystem turretSubsystem,
        Supplier<Pose2d> drivetrainPose,
        Supplier<ChassisSpeeds> drivetrainVelocity) {
      this.shooterSubsystem = shooterSubsystem;
      this.hoodSubsystem = hoodSubsystem;
      this.hopperFloorSubsystem = hopperFloorSubsystem;
      this.turretSubsystem = turretSubsystem;
      this.drivetrainPose = drivetrainPose;
      this.drivetrainVelocity = drivetrainVelocity;
      addRequirements(shooterSubsystem, hoodSubsystem, hopperFloorSubsystem, turretSubsystem);
      setName("ShootOnTheMoveCommand");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      Translation3d target;

      if (FieldCoordinates.BLUE_BUMP_AND_TRENCH_X <= drivetrainPose.get().getX()
          && drivetrainPose.get().getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
        if (FieldCoordinates.FIELD_CENTER.getY() < drivetrainPose.get().getY()) {
          target =
              DriverStation.getAlliance().get() == Alliance.Blue
                  ? FieldCoordinates.BLUE_RIGHT_FUNNELING
                  : FieldCoordinates.RED_LEFT_FUNNELING;

        } else {
          target =
              DriverStation.getAlliance().get() == Alliance.Blue
                  ? FieldCoordinates.BLUE_LEFT_FUNNELING
                  : FieldCoordinates.RED_RIGHT_FUNNELING;
        }

      } else {
        target =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? FieldCoordinates.BLUE_HUB
                : FieldCoordinates.RED_HUB;
      }

      ShotParameters params =
          ShotCalculator.calculateShot(
              drivetrainPose.get(),
              drivetrainVelocity.get(),
              target); // calculates the shot params with turretAngle
      double desiredRotorVelocity = ShooterSubsystem.launchSpeedToRotorSpeed(params.shotSpeed);

      Logger.recordOutput("Mech/ShootingCommand/validShot", params.shotSpeed != 0);
      Logger.recordOutput("Mech/ShootingCommand/shotSpeed", params.shotSpeed);
      Logger.recordOutput("Mech/ShootingCommand/rotorSpeed", desiredRotorVelocity);

      Logger.recordOutput("Mech/ShootingCommand/hoodAngle", params.hoodAngle.getDegrees());
      Logger.recordOutput("Mech/ShootingCommand/turretAngle", params.turretAngle.getDegrees());
      Logger.recordOutput("Mech/ShootingCommand/currentPose", drivetrainPose.get());
      Logger.recordOutput("Mech/ShootingCommand/chassisSpeeds", drivetrainVelocity.get());

      Logger.recordOutput("Mech/ShootingCommand/target", target);
      Logger.recordOutput(
          "Mech/ShootingCommand/Current alliance", DriverStation.getAlliance().get());

      System.out.println("SHOOTING ON THE MOVE TARGET:" + target);

      if (params.shotSpeed == 0) { // if we dont have a valid shot
        shooterSubsystem.setDesiredRotorVelocity(0);
        shooterSubsystem.setDesiredTransitionVoltage(0);
        hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
      } else {
        turretSubsystem.setDesiredAngle(params.turretAngle);
        shooterSubsystem.setDesiredRotorVelocity(
            params.shotSpeed); // set velocity to our desired velocity
        hopperFloorSubsystem.setDesiredHopperFloorVoltage(
            HopperFloorConstants.HOPPER_FLOOR_VOLTAGE);
        if (Math.abs(shooterSubsystem.getFlywheelRotorVelocity() - params.shotSpeed)
            < ShooterConstants.FLYWHEEL_SPEED_DEADBAND) { // once flywheel is running close to
          // our desired velocity
          hoodSubsystem.setDesiredAngle(
              hoodSubsystem.convertLaunchAngleToHoodAngle(params.hoodAngle));
          shooterSubsystem.setDesiredTransitionVoltage(ShooterConstants.TRANSITION_VOLTAGE);
        }
      }
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public void end(boolean interrupted) {
      hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
      shooterSubsystem.setDesiredTransitionVoltage(0);
      shooterSubsystem.setDesiredRotorVelocity(0);
    }
  }
}
