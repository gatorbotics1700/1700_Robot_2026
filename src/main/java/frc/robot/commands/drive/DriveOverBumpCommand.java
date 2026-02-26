package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.ShooterSubsystem;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import org.json.simple.parser.ParseException;

// TODO: make sure we have field avoidance file
public class DriveOverBumpCommand {

  public static Command driveOverBump(Drive drive, ShooterSubsystem shooterSubsystem)
      throws IOException, ParseException {
    PathConstraints constraints =
        new PathConstraints(8, 8, Units.degreesToRadians(700), Units.degreesToRadians(1000));

    BooleanSupplier shouldShootTrue =
        () -> {
          return true;
        };
    BooleanSupplier shouldShootFalse =
        () -> {
          return false;
        };
    Pose2d pose = drive.getPose();
    System.out.println("*********************DRIVE OVER BUMP COMMAND*******************");
    Command pathToFollow;

    if (pose.getX() <= FieldCoordinates.FIELD_CENTER.getX()) { // BLUE
      if (pose.getY() < FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.BLUE_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B BR N to A"), constraints);
        shooterSubsystem.setShouldShoot(shouldShootTrue);
      } else if (pose.getY() > FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.BLUE_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B BL N to A"), constraints);
        shooterSubsystem.setShouldShoot(shouldShootTrue);
      } else if ((pose.getY() > FieldCoordinates.FIELD_CENTER.getY())
          && (pose.getX() > FieldCoordinates.BLUE_BUMP_AND_TRENCH_X)) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B BL A to N"), constraints);
        shooterSubsystem.setShouldShoot(shouldShootFalse);
      } else {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B BR A to N"), constraints);
        shooterSubsystem.setShouldShoot(shouldShootFalse);
      }
    } else { // RED
      if (pose.getY() < FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R BL A to N"), constraints);
        shooterSubsystem.setShouldShoot(shouldShootFalse);
      } else if (pose.getY() > FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R BR A to N"), constraints);
        shooterSubsystem.setShouldShoot(shouldShootFalse);
      } else if ((pose.getY() > FieldCoordinates.FIELD_CENTER.getY())
          && (pose.getX() > FieldCoordinates.RED_BUMP_AND_TRENCH_X)) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R BR N to A"), constraints);
        shooterSubsystem.setShouldShoot(shouldShootTrue);
      } else {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R BL N to A"), constraints);
        shooterSubsystem.setShouldShoot(shouldShootTrue);
      }
    }
    return pathToFollow; // TODO: stop shooting and then path to follow -- need to add part to stop
    // shooting
  }
}
