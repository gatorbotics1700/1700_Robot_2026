package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.commands.mech.ShootingCommands;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class DynamicAutoBuilder {

  private final IntakeSubsystem intakeSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final Supplier<Pose2d> robotPose;

  public DynamicAutoBuilder(
      IntakeSubsystem intakeSubsystem,
      HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> robotPose) {
    this.intakeSubsystem = intakeSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.hopperFloorSubsystem = hopperFloorSubsystem;
    this.robotPose = robotPose;
  }

  private String convertFromLocation(String location) {
    // Path files use "Center" not "C" for the hub/center position
    if (location != null && location.equals("C")) {
      return "Center";
    }
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
    String convertedTo = convertFromLocation(to);
    return alliance + " " + convertedFrom + " to " + convertedTo;
  }

  private static final double HUB_SHOOT_DURATION_SECONDS = 3.0;

  /** Checks if the location is the Hub/Center shooting position. */
  private boolean isHub(String location) {
    return location != null && location.equals("C");
  }

  /** Checks if the location is the Center start position. */
  private boolean isCenterStart(String location) {
    return location != null && location.equals("Center");
  }

  /** Creates a 3-second shooting command at the Hub position. */
  private Command createHubShootCommand() {
    return ShootingCommands.StationaryShootingCommand(
            shooterSubsystem, hoodSubsystem, hopperFloorSubsystem, robotPose)
        .withTimeout(HUB_SHOOT_DURATION_SECONDS);
  }

  private Command getActionForDestination(String destination) {
    if (destination == null || destination.equals("None")) {
      return Commands.none();
    }

    if (destination.startsWith("Fuel Pile") && RobotBase.isReal()) {
      return new HoodCommands.HoodRetractCommand(hoodSubsystem);
    }

    return Commands.none();
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
      // Use pathfindThenFollowPath to avoid obstacles like the trench/hub
      return AutoBuilder.pathfindThenFollowPath(path, path.getGlobalConstraints());
    } catch (Exception e) {
      System.out.println("  Path not available: " + pathName + " - " + e.getMessage());
      return Commands.none();
    }
  }

  public Command buildAuto(
      String alliance, String startPos, String dest1, String dest2, String dest3) {

    System.out.println("============BUILD AUTO============");
    System.out.println("Alliance: " + alliance);
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

    // If starting at Center, shoot first for 3 seconds
    if (isCenterStart(startPos) && RobotBase.isReal()) {
      System.out.println("  Starting at Center - adding initial 3 second shoot");
      commandSequence.add(createHubShootCommand());
    }

    // Build paths with shooting at Hub positions
    String[] destinations = {dest1, dest2, dest3};

    for (String destination : destinations) {
      if (destination == null || destination.equals("None")) {
        break;
      }

      // Load and add the path
      Command pathCmd = loadPathCommand(alliance, currentLocation, destination);
      Command pathAction = getActionForDestination(destination);

      if (RobotBase.isReal()) {
        // Run intake while driving
        Command driveWithIntake =
            pathCmd.deadlineFor(
                IntakeCommands.DeployIntake(intakeSubsystem)
                    .alongWith(IntakeCommands.RunIntake(intakeSubsystem))
                    .alongWith(pathAction));
        commandSequence.add(driveWithIntake);
      } else {
        // In sim, just run the path without mech commands
        commandSequence.add(pathCmd);
      }

      // If destination is Hub (C), shoot for 3 seconds after arriving
      if (isHub(destination) && RobotBase.isReal()) {
        System.out.println(" Arrived at Hub - adding 3 second shoot");
        commandSequence.add(createHubShootCommand());
      }

      currentLocation = destination;
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
