package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.mech.ShootingCommands.ShootOnTheMoveCommand;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class DynamicAutoBuilder {

  private final HoodSubsystem hoodSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  public DynamicAutoBuilder(
     // IntakeSubsystem intakeSubsystem,
      HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.hopperFloorSubsystem = hopperFloorSubsystem;
    this.robotPose = robotPose;
    this.chassisSpeeds = chassisSpeeds;
  }

  private String convertFromLocation(String location) {
    return location;
  }

  private boolean isInvalid(String value) {
    return value == null || value.equals("None");
  }

  private String buildPathName(String alliance, String from, String to) {
    // Need all three values to build a path name
    if (isInvalid(alliance) || isInvalid(from) || isInvalid(to)) {
      return null;
    }

    String convertedFrom = convertFromLocation(from);
    return alliance + " " + convertedFrom + " to " + to;
  }

  /** Checks if the location is center (C or Center). */
  private boolean isCenter(String location) {
    return location != null && (location.equals("C") || location.equalsIgnoreCase("Center"));
  }

  /** Creates shooting command. */
  private Command createShootingCommand() {
    return Commands.runOnce(() -> shooterSubsystem.setShouldShoot(true))
        .andThen(
            new ShootOnTheMoveCommand(
                shooterSubsystem,
                hoodSubsystem,
                turretSubsystem,
                hopperFloorSubsystem,
                robotPose,
                chassisSpeeds))
        .finallyDo(() -> shooterSubsystem.setShouldShoot(false));
  }

  private Command loadPathCommand(String alliance, String from, String to) {
    String pathName = buildPathName(alliance, from, to);
    if (pathName == null) {
      System.out.println("  Could not build path name for: " + alliance + " " + from + " to " + to);
      return Commands.none();
    }

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      System.out.println("  Loaded path: " + pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.println("  Path not available: " + pathName + " - " + e.getMessage());
      return Commands.none();
    }
  }

  public Command buildAuto(
      String alliance, String startPos, String dest1, String dest2, String dest3) {

    System.out.println("============BUILD AUTO============");
    System.out.println("Alliance: " + alliance);
    System.out.println("Start Position: " + startPos);
    System.out.println("First Destination: " + dest1);
    System.out.println("Second Destination: " + dest2);
    System.out.println("Third Destination: " + dest3);

    if (alliance == null || alliance.equals("None")) {
      System.out.println("DynamicAutoBuilder: Missing alliance");
      return Commands.none();
    }
    if (startPos == null || startPos.equals("None")) {
      System.out.println("DynamicAutoBuilder: Missing start position");
      return Commands.none();
    }

    boolean hasDestination = dest1 != null && !dest1.equals("None");
    if (!hasDestination) {
      System.out.println("DynamicAutoBuilder: No destinations selected");
      return Commands.none();
    }

    System.out.println(
        "DynamicAutoBuilder: Building auto - "
            + alliance
            + " "
            + startPos
            + " -> "
            + dest1
            + " -> "
            + dest2
            + " -> "
            + dest3);

    List<Command> commandSequence = new ArrayList<>();
    String currentLocation = startPos;

    // If starting at center, pause for 3 seconds then shoot for 3 seconds
    if (isCenter(startPos)) {
      System.out.println("  Starting at center - will pause 3s then shoot for 3s");
      commandSequence.add(
          Commands.waitSeconds(3.0)
              .andThen(createShootingCommand().withDeadline(Commands.waitSeconds(3.0))));
    }

    // Build paths with shooting when destination is center
    if (dest1 != null && !dest1.equals("None")) {
      Command firstPath = loadPathCommand(alliance, currentLocation, dest1);

      // Run path
      commandSequence.add(firstPath);

      if (isCenter(dest1)) {
        // Going to center - shoot for 3 seconds when we arrive
        commandSequence.add(createShootingCommand().withDeadline(Commands.waitSeconds(3.0)));
      }
      currentLocation = dest1;

      if (dest2 != null && !dest2.equals("None")) {
        Command secondPath = loadPathCommand(alliance, currentLocation, dest2);

        // Run path
        commandSequence.add(secondPath);

        if (isCenter(dest2)) {
          // Going to center - shoot for 3 seconds when we arrive
          commandSequence.add(createShootingCommand().withDeadline(Commands.waitSeconds(3.0)));
        }
        currentLocation = dest2;

        if (dest3 != null && !dest3.equals("None")) {
          Command thirdPath = loadPathCommand(alliance, currentLocation, dest3);

          // Run path
          commandSequence.add(thirdPath);

          if (isCenter(dest3)) {
            // Going to center - shoot for 3 seconds when we arrive
            commandSequence.add(createShootingCommand().withDeadline(Commands.waitSeconds(3.0)));
          }
        }
      }
    }

    if (commandSequence.isEmpty()) {
      System.out.println("DynamicAutoBuilder: No paths loaded - running empty auto");
      return Commands.none();
    }

    return Commands.sequence(commandSequence.toArray(new Command[0]));
  }

  public Optional<String> getFirstPathName(
      String alliance, String startPos, String dest1, String dest2, String dest3) {
    if (isInvalid(alliance) || isInvalid(startPos)) {
      return Optional.empty();
    }
    if (dest1 == null || dest1.equals("None")) {
      return Optional.empty();
    }
    String name = buildPathName(alliance, startPos, dest1);
    return name != null ? Optional.of(name) : Optional.empty();
  }
}
