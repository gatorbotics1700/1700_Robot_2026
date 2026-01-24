package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Multi-step auto chooser that allows drivers to select alliance, starting position, and path
 * segments step-by-step. The chooser automatically builds the auto command from standardized path
 * names.
 */
public class MultiStepAutoChooser {
  private final LoggedDashboardChooser<String> allianceChooser;
  private final LoggedDashboardChooser<String> startPosChooser;
  private final LoggedDashboardChooser<String> destinationChooser;
  private final LoggedDashboardChooser<String> nextDestinationChooser;

  private final Set<String> availablePaths = new HashSet<>();
  private final List<PathSegment> pathSegments = new ArrayList<>();

  /** Represents a path segment with alliance, start position, and destination. */
  private static class PathSegment {
    final String alliance;
    final String startPos;
    final String destination;
    final String pathName;

    PathSegment(String alliance, String startPos, String destination, String pathName) {
      this.alliance = alliance;
      this.startPos = startPos;
      this.destination = destination;
      this.pathName = pathName;
    }
  }

  public MultiStepAutoChooser() {
    // Load all available paths
    loadAvailablePaths();

    // Initialize choosers
    allianceChooser = new LoggedDashboardChooser<>("Auto/Alliance");
    startPosChooser = new LoggedDashboardChooser<>("Auto/Start Position");
    destinationChooser = new LoggedDashboardChooser<>("Auto/Destination");
    nextDestinationChooser = new LoggedDashboardChooser<>("Auto/Next Destination");

    // Populate alliance chooser
    Set<String> alliances =
        pathSegments.stream().map(seg -> seg.alliance).collect(Collectors.toSet());
    allianceChooser.addDefaultOption("None", "None");
    for (String alliance : alliances) {
      allianceChooser.addOption(alliance.equals("R") ? "Red" : "Blue", alliance);
    }

    // Populate start position chooser with all possible options
    // (Driver will need to select valid combinations)
    Set<String> allStartPositions =
        pathSegments.stream().map(seg -> seg.startPos).collect(Collectors.toSet());
    startPosChooser.addDefaultOption("None", "None");
    for (String startPos : allStartPositions) {
      startPosChooser.addOption(startPos, startPos);
    }

    // Populate destination chooser with all possible options
    Set<String> allDestinations =
        pathSegments.stream().map(seg -> seg.destination).collect(Collectors.toSet());
    destinationChooser.addDefaultOption("None", "None");
    for (String destination : allDestinations) {
      destinationChooser.addOption(destination, destination);
    }

    // Populate next destination chooser (for multi-segment paths)
    nextDestinationChooser.addDefaultOption("None (End)", "None");
    for (String destination : allDestinations) {
      nextDestinationChooser.addOption(destination, destination);
    }

    // Force initialization by reading each chooser once
    // This ensures they publish to NetworkTables
    allianceChooser.get();
    startPosChooser.get();
    destinationChooser.get();
    nextDestinationChooser.get();
  }

  /** Loads all available paths from the deploy directory and parses their names. */
  private void loadAvailablePaths() {
    File pathsDir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");

    if (!pathsDir.exists() || !pathsDir.isDirectory()) {
      System.err.println("MultiStepAutoChooser: Paths directory not found: " + pathsDir.getPath());
      return;
    }

    File[] pathFiles = pathsDir.listFiles((dir, name) -> name.endsWith(".path"));
    if (pathFiles == null) {
      System.err.println("MultiStepAutoChooser: No path files found");
      return;
    }

    for (File pathFile : pathFiles) {
      String fileName = pathFile.getName();
      // Remove .path extension
      String pathName = fileName.substring(0, fileName.length() - 5);
      availablePaths.add(pathName);

      // Parse path name: Format is "{Alliance} {StartPos} to {Destination}"
      // Examples: "R Center to DC", "B LF to Fuel Pile CF", "R RF to Outpost"
      PathSegment segment = parsePathName(pathName);
      if (segment != null) {
        pathSegments.add(segment);
      }
    }

    System.out.println("MultiStepAutoChooser: Loaded " + pathSegments.size() + " path segments");
  }

  /**
   * Parses a path name into its components.
   *
   * @param pathName The path name to parse (e.g., "R Center to DC", "B LF to Fuel Pile CF")
   * @return A PathSegment if parsing succeeds, null otherwise
   */
  private PathSegment parsePathName(String pathName) {
    // Handle special case: "Stay" paths (e.g., "B Center Stay")
    if (pathName.endsWith(" Stay")) {
      String prefix = pathName.substring(0, pathName.length() - 5).trim();
      String[] parts = prefix.split(" ", 2);
      if (parts.length == 2) {
        return new PathSegment(parts[0], parts[1], "Stay", pathName);
      }
      return null;
    }

    // Standard format: "{Alliance} {StartPos} to {Destination}"
    if (!pathName.contains(" to ")) {
      return null;
    }

    String[] mainParts = pathName.split(" to ", 2);
    if (mainParts.length != 2) {
      return null;
    }

    String startPart = mainParts[0].trim();
    String destination = mainParts[1].trim();

    // Parse start part: "{Alliance} {StartPos}"
    String[] startParts = startPart.split(" ", 2);
    if (startParts.length != 2) {
      return null;
    }

    String alliance = startParts[0].trim();
    String startPos = startParts[1].trim();

    return new PathSegment(alliance, startPos, destination, pathName);
  }

  /**
   * Updates chooser options based on current selections. Should be called periodically. Note: Since
   * LoggedDashboardChooser doesn't support dynamic option clearing, all options are populated
   * upfront. This method is kept for potential future enhancements.
   *
   * <p>Reading the choosers here ensures they stay published to NetworkTables.
   */
  public void updateChooserOptions() {
    // Read choosers to ensure they stay published to NetworkTables
    // This is necessary for LoggedDashboardChooser to maintain NetworkTables entries
    allianceChooser.get();
    startPosChooser.get();
    destinationChooser.get();
    nextDestinationChooser.get();
  }

  /**
   * Gets the autonomous command based on the current selections.
   *
   * @return The command to run in autonomous, or Commands.none() if no valid selection
   */
  public Command getAutonomousCommand() {
    try {
      // Update chooser options first
      updateChooserOptions();

      String alliance = allianceChooser.get();
      String startPos = startPosChooser.get();
      String destination = destinationChooser.get();
      String nextDestination = nextDestinationChooser.get();

      // Build command from selections
      List<Command> pathCommands = new ArrayList<>();
      List<String> selectedPathNames = new ArrayList<>();

      // First segment: from start position to destination
      if (alliance != null
          && !alliance.equals("None")
          && startPos != null
          && !startPos.equals("None")
          && destination != null
          && !destination.equals("None")) {

        String firstPathName = findPathName(alliance, startPos, destination);
        if (firstPathName != null) {
          try {
            PathPlannerPath firstPath = PathPlannerPath.fromPathFile(firstPathName);
            if (firstPath != null) {
              pathCommands.add(AutoBuilder.followPath(firstPath));
              selectedPathNames.add(firstPathName);
            } else {
              System.err.println("MultiStepAutoChooser: Could not load path: " + firstPathName);
            }
          } catch (Exception e) {
            System.err.println(
                "MultiStepAutoChooser: Error loading path "
                    + firstPathName
                    + ": "
                    + e.getMessage());
          }
        } else {
          System.err.println(
              "MultiStepAutoChooser: Path not found for: "
                  + alliance
                  + " "
                  + startPos
                  + " to "
                  + destination);
        }
      }

      // Second segment: from destination to next destination (if selected)
      if (nextDestination != null
          && !nextDestination.equals("None")
          && destination != null
          && !destination.equals("None")) {

        // Try to find a path from the current destination to the next destination
        // The destination from the first segment should become the start position for the second
        // First try exact match: destination as start position
        String secondPathName = findPathName(alliance, destination, nextDestination);
        if (secondPathName == null) {
          // Try to find any path that starts from a position matching the destination
          // and goes to the next destination
          secondPathName = findPathNameByDestination(alliance, destination, nextDestination);
        }

        if (secondPathName != null) {
          try {
            PathPlannerPath secondPath = PathPlannerPath.fromPathFile(secondPathName);
            if (secondPath != null) {
              pathCommands.add(AutoBuilder.followPath(secondPath));
              selectedPathNames.add(secondPathName);
            } else {
              System.err.println("MultiStepAutoChooser: Could not load path: " + secondPathName);
            }
          } catch (Exception e) {
            System.err.println(
                "MultiStepAutoChooser: Error loading path "
                    + secondPathName
                    + ": "
                    + e.getMessage());
          }
        } else {
          System.err.println(
              "MultiStepAutoChooser: Could not find path from "
                  + destination
                  + " to "
                  + nextDestination);
        }
      }


      if (pathCommands.isEmpty()) {
        return Commands.none();
      }

      // Combine all path commands sequentially
      return Commands.sequence(pathCommands.toArray(new Command[0]));
    } catch (Exception e) {
      System.err.println("MultiStepAutoChooser: Error building auto command: " + e.getMessage());
      e.printStackTrace();
      return Commands.none();
    }
  }

  /** Finds a path name matching the given alliance, start position, and destination. */
  private String findPathName(String alliance, String startPos, String destination) {
    return pathSegments.stream()
        .filter(
            seg ->
                seg.alliance.equals(alliance)
                    && seg.startPos.equals(startPos)
                    && seg.destination.equals(destination))
        .map(seg -> seg.pathName)
        .findFirst()
        .orElse(null);
  }

  /** Attempts to find a path by matching destination patterns (for multi-segment paths). */
  private String findPathNameByDestination(
      String alliance, String fromDestination, String toDestination) {
    // This is a fallback - tries to find paths where the start position might match
    // the destination name in some way
    return pathSegments.stream()
        .filter(seg -> seg.alliance.equals(alliance))
        .filter(
            seg -> {
              // Try to match start position with the from destination
              return seg.startPos.contains(fromDestination)
                  || fromDestination.contains(seg.startPos);
            })
        .filter(seg -> seg.destination.equals(toDestination))
        .map(seg -> seg.pathName)
        .findFirst()
        .orElse(null);
  }
}
