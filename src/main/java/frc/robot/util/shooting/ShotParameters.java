package frc.robot.util.shooting;

import edu.wpi.first.math.geometry.Pose2d;

// @AutoLog
public class ShotParameters {
  public double turretAngle; // robot relative
  public double hoodAngle; // degrees from vertical
  public double shotSpeed; // mps of the ball's exit velo
  public Pose2d pose; // robot pose field relative

  // (note: shotSpeed is NOT the same as the kraken's speed)

  public ShotParameters(double turretAngle, double hoodAngle, double shotSpeed) {
    this.turretAngle = turretAngle;
    this.hoodAngle = hoodAngle;
    this.shotSpeed = shotSpeed;
  }
}
