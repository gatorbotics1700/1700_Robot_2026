package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class FunnelingCommand extends Command {
  private static Pose2d currentPose;
  private Drive drive;

  private static final double MIN_Y_BOUND = 20.0;
  private static final double MAX_Y_BOUND = 30.0;

  private final Translation3d targetRight = new Translation3d(10.0, 50.0, 0.0);
  private final Translation3d targetLeft = new Translation3d(10.0, 10.0, 0.0);
  private final Translation3d targetMiddle = new Translation3d(10.0, 25.0, 0.0);

  public FunnelingCommand() {
    currentPose = drive.getPose();
  }

  @Override
  public void execute() {
    currentPose = drive.getPose();
    if (currentPose.getY() > MIN_Y_BOUND
        && currentPose.getY() < MAX_Y_BOUND) { // if within range where hub is blocking
      // then shoot OR shoot really high?
    }
  }
}
