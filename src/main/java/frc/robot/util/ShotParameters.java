package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

// @AutoLog
public class ShotParameters {
  public Rotation2d turretAngle;
  public Rotation2d hoodAngle;
  public Translation3d trajectoryRelativeExitVelo;
  public Rotation2d fieldRelativeTurretAngle;

  public ShotParameters(Rotation2d turretAngle, Rotation2d hoodAngle) {
    this.turretAngle = turretAngle;
    this.hoodAngle = hoodAngle;
  }
}
