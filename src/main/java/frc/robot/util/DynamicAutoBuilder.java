package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.commands.mech.ShootingCommands.ShootOnTheMoveCommand;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class DynamicAutoBuilder {

  private final IntakeSubsystem intakeSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  public DynamicAutoBuilder(
      IntakeSubsystem intakeSubsystem,
      HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.intakeSubsystem = intakeSubsystem;
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
    return location != null
        && (location.equals("C") || location.equalsIgnoreCase("Center"));
  }

  /** Creates shooting command. */
  private Command createShootingCommand() {
    return new ShootOnTheMoveCommand(
        shooterSubsystem,
        hoodSubsystem,
        turretSubsystem,
        hopperFloorSubsystem,
        robotPose,
        chassisSpeeds);
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

    // If starting at center, pause for 3 seconds then shoot
    if (isCenter(startPos) && RobotBase.isReal()) {
      System.out.println("  Starting at center - will pause 3s then shoot");
      commandSequence.add(
          Commands.waitSeconds(3.0)
              .andThen(createShootingCommand()));
    }

    // Build paths with shooting when destination is center
    if (dest1 != null && !dest1.equals("None")) {
      Command firstPath = loadPathCommand(alliance, currentLocation, dest1);
      Command firstAction = getActionForDestination(dest1);

      if (isCenter(dest1) && RobotBase.isReal()) {
        // Going to center - run path with intake, then shoot when we arrive
        Command pathWithIntake = firstPath.deadlineFor(
            IntakeCommands.DeployIntake(intakeSubsystem)
                .alongWith(IntakeCommands.RunIntake(intakeSubsystem))
                .alongWith(firstAction));
        commandSequence.add(pathWithIntake);
        commandSequence.add(createShootingCommand());
      } else {
        // Not going to center - just run path with intake
        if (RobotBase.isReal()) {
          Command pathWithIntake = firstPath.deadlineFor(
              IntakeCommands.DeployIntake(intakeSubsystem)
                  .alongWith(IntakeCommands.RunIntake(intakeSubsystem))
                  .alongWith(firstAction));
          commandSequence.add(pathWithIntake);
        } else {
          commandSequence.add(firstPath.deadlineFor(firstAction));
        }
      }
      currentLocation = dest1;

      if (dest2 != null && !dest2.equals("None")) {
        Command secondPath = loadPathCommand(alliance, currentLocation, dest2);
        Command secondAction = getActionForDestination(dest2);

        if (isCenter(dest2) && RobotBase.isReal()) {
          Command pathWithIntake = secondPath.deadlineFor(
              IntakeCommands.RunIntake(intakeSubsystem)
                  .alongWith(secondAction));
          commandSequence.add(pathWithIntake);
          commandSequence.add(createShootingCommand());
        } else {
          if (RobotBase.isReal()) {
            Command pathWithIntake = secondPath.deadlineFor(
                IntakeCommands.RunIntake(intakeSubsystem)
                    .alongWith(secondAction));
            commandSequence.add(pathWithIntake);
          } else {
            commandSequence.add(secondPath.deadlineFor(secondAction));
          }
        }
        currentLocation = dest2;

        if (dest3 != null && !dest3.equals("None")) {
          Command thirdPath = loadPathCommand(alliance, currentLocation, dest3);
          Command thirdAction = getActionForDestination(dest3);

          if (isCenter(dest3) && RobotBase.isReal()) {
            Command pathWithIntake = thirdPath.deadlineFor(
                IntakeCommands.RunIntake(intakeSubsystem)
                    .alongWith(thirdAction));
            commandSequence.add(pathWithIntake);
            commandSequence.add(createShootingCommand());
          } else {
            if (RobotBase.isReal()) {
              Command pathWithIntake = thirdPath.deadlineFor(
                  IntakeCommands.RunIntake(intakeSubsystem)
                      .alongWith(thirdAction));
              commandSequence.add(pathWithIntake);
            } else {
              commandSequence.add(thirdPath.deadlineFor(thirdAction));
            }
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
