package frc.robot.CSPLib.ppp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.FieldConstants;
import java.util.*;

/**
 * Node-based A* generator using predefined sparse field graph.
 *
 * <p>Returns: - currentRobotPosition as first element - intermediate graph nodes - endPosition as
 * last element
 *
 * <p>Designed for fast real-time execution.
 */
public final class NodePathGenerator {

  private NodePathGenerator() {}

  /* ================= NAV NODE ================= */

  private static class NavNode {
    final String name;
    final Translation2d position;
    final List<String> linkNames;
    final List<NavNode> neighbors = new ArrayList<>();

    NavNode(String name, double x, double y, String... links) {
      this.name = name;
      this.position = new Translation2d(x, y);
      this.linkNames = Arrays.asList(links);
    }
  }

  /* ================= GRAPH ================= */

  private static final Map<String, NavNode> graph = new HashMap<>();

  static {
    initializeGraph();
    resolveLinks();
  }

  private static void initializeGraph() {

    add(
        "field_center",
        8.270,
        4.035,
        "right_trench_neutral_preentrance",
        "left_trench_neutral_preentrance");

    add(
        "right_trench_center",
        4.651,
        0.639,
        "right_trench_neutral_entrance",
        "right_trench_alliance_entrance");

    add(
        "right_trench_neutral_entrance",
        5.186,
        0.639,
        "right_trench_neutral_preentrance",
        "right_trench_center");

    add(
        "right_trench_alliance_entrance",
        3.978,
        0.639,
        "right_trench_alliance_preentrance",
        "right_trench_center");

    add(
        "right_trench_neutral_preentrance",
        5.686,
        0.639,
        "field_center",
        "right_trench_neutral_entrance");

    add(
        "right_trench_alliance_preentrance",
        3.478,
        0.639,
        "right_trench_alliance_entrance",
        "right_alliance_shoot");

    add(
        "left_trench_center",
        4.651,
        7.430,
        "left_trench_neutral_entrance",
        "left_trench_alliance_entrance");

    add(
        "left_trench_neutral_entrance",
        5.186,
        7.430,
        "left_trench_neutral_preentrance",
        "left_trench_center");

    add(
        "left_trench_alliance_entrance",
        3.978,
        7.430,
        "left_trench_alliance_preentrance",
        "left_trench_center");

    add(
        "left_trench_neutral_preentrance",
        5.686,
        7.430,
        "field_center",
        "left_trench_neutral_entrance");

    add(
        "left_trench_alliance_preentrance",
        3.478,
        7.430,
        "left_trench_alliance_entrance",
        "left_alliance_shoot");

    add(
        "right_alliance_shoot",
        3.386,
        2.018,
        "center_alliance_shoot",
        "right_trench_alliance_preentrance");

    add(
        "left_alliance_shoot",
        3.386,
        6.052,
        "center_alliance_shoot",
        "left_trench_alliance_preentrance");

    add("center_alliance_shoot", 3.386, 4.035, "right_alliance_shoot", "left_alliance_shoot");

    add(
        "fuelfield_right",
        FieldConstants.FuelField.right_close_corner.getX(),
        FieldConstants.FuelField.right_close_corner.getY(),
        "field_center",
        "right_trench_neutral_preentrance");

    add(
        "fuelfield_left",
        FieldConstants.FuelField.left_close_corner.getX(),
        FieldConstants.FuelField.left_close_corner.getY(),
        "field_center",
        "left_trench_neutral_preentrance");
  }

  private static void add(String name, double x, double y, String... links) {
    graph.put(name, new NavNode(name, x, y, links));
  }

  private static void resolveLinks() {
    for (NavNode node : graph.values()) {
      for (String link : node.linkNames) {
        NavNode neighbor = graph.get(link);
        if (neighbor != null) {
          node.neighbors.add(neighbor);
        }
      }
    }
  }

  /* ================= PUBLIC METHOD ================= */

  public static List<Translation2d> generateNodePath(
      Translation2d currentRobotPosition, Translation2d endPosition) {

    NavNode startNode = nearestNode(currentRobotPosition);
    NavNode goalNode = nearestNode(endPosition);

    PriorityQueue<AStarNode> open = new PriorityQueue<>();
    Map<NavNode, Double> gCosts = new HashMap<>();

    open.add(new AStarNode(startNode, null, 0, heuristic(startNode.position, goalNode.position)));

    gCosts.put(startNode, 0.0);

    AStarNode goalResult = null;

    while (!open.isEmpty()) {

      AStarNode current = open.poll();

      if (current.node == goalNode) {
        goalResult = current;
        break;
      }

      for (NavNode neighbor : current.node.neighbors) {

        double tentativeG = current.gCost + current.node.position.getDistance(neighbor.position);

        if (!gCosts.containsKey(neighbor) || tentativeG < gCosts.get(neighbor)) {

          gCosts.put(neighbor, tentativeG);

          double f = tentativeG + heuristic(neighbor.position, goalNode.position);

          open.add(new AStarNode(neighbor, current, tentativeG, f));
        }
      }
    }

    List<Translation2d> result = new ArrayList<>();
    result.add(currentRobotPosition);

    if (goalResult != null) {
      List<Translation2d> reversed = new ArrayList<>();
      AStarNode node = goalResult;

      while (node != null) {
        reversed.add(node.node.position);
        node = node.parent;
      }

      Collections.reverse(reversed);

      // Remove duplicate start if extremely close
      if (!reversed.isEmpty() && reversed.get(0).getDistance(currentRobotPosition) < 0.01) {
        reversed.remove(0);
      }

      result.addAll(reversed);
    }

    result.add(endPosition);

    return result;
  }

  /**
   * Generates a node-based A* path and returns it as Pose2d objects with Rotation2d.kZero for every
   * waypoint.
   *
   * @param currentRobotPosition Current robot pose
   * @param endPosition Target end pose
   * @return List of Pose2d where: - First element = currentRobotPosition - Last element =
   *     endPosition (translation only, rotation = kZero) - Intermediate elements = node waypoints
   */
  public static List<Pose2d> generateNodePathWithPose2d(
      Translation2d currentRobotPosition, Translation2d endPosition) {
    // Reuse your existing Translation-based A* implementation
    List<Translation2d> translationPath = generateNodePath(currentRobotPosition, endPosition);

    List<Pose2d> posePath = new ArrayList<>();

    for (Translation2d translation : translationPath) {
      posePath.add(new Pose2d(translation, Rotation2d.kZero));
    }

    return posePath;
  }

  /* ================= HELPERS ================= */

  private static NavNode nearestNode(Translation2d position) {
    NavNode best = null;
    double bestDist = Double.MAX_VALUE;

    for (NavNode node : graph.values()) {
      double dist = node.position.getDistance(position);
      if (dist < bestDist) {
        bestDist = dist;
        best = node;
      }
    }
    return best;
  }

  private static double heuristic(Translation2d a, Translation2d b) {
    return a.getDistance(b);
  }

  private static class AStarNode implements Comparable<AStarNode> {
    final NavNode node;
    final AStarNode parent;
    final double gCost;
    final double fCost;

    AStarNode(NavNode node, AStarNode parent, double gCost, double fCost) {
      this.node = node;
      this.parent = parent;
      this.gCost = gCost;
      this.fCost = fCost;
    }

    @Override
    public int compareTo(AStarNode other) {
      return Double.compare(this.fCost, other.fCost);
    }
  }
}
