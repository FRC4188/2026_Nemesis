package frc.robot.CSPLib.ppp;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.*;

public final class NodePathGenerator {
  private NodePathGenerator() {}

  // Tunables (my demise)
  private static final double COARSE_STEP_DEG = 30.0;
  private static final double FINE_STEP_DEG = 1.0;
  private static final double ANGLE_RANGE_DEG = 360.0;

  private static final double SAMPLE_DT = 0.02; // seconds
  private static final double AUTO_CONTROL_DISTANCE_FACTOR = 0.25;
  private static final double CURVATURE_WEIGHT = 2.0;
  private static final double SEGMENT_SAMPLE_METERS = 0.05;
  private static final double CURV_EPS_DIST = 1e-4;

  private static final double PATH_ERROR_METERS = 0.5;

  // Other configs
  private static boolean runBefore = false;
  private static Pose2d lastPose = null;

  private static PathConstraints globalConstraints = null;
  private static RobotConfig globalConfig = null;
  private static final List<NavNode> navNodes = new ArrayList<>();
  private static final Map<String, NavNode> navNodeMap = new HashMap<>();
  private static final List<RectObstacle> obstacles = new ArrayList<>();
  private static final List<RectObstacle> inflated = new ArrayList<>();
  private static double robotInflationRadius = 0.0;

  // Configuring methods, sets up a lot of the heuristics later on

  public static void configureConstraints(PathConstraints constraints, RobotConfig config) {
    globalConstraints = Objects.requireNonNull(constraints, "PathConstraints required");
    globalConfig = Objects.requireNonNull(config, "RobotConfig required");
  }

  /**
   * Configure graph nodes directly. Each NavNode has: id, pose, and list of linked node ids
   * (links).
   */
  public static void configureNodes(NavNode... nodes) {
    navNodes.clear();
    navNodeMap.clear();
    if (nodes == null) return;
    for (NavNode n : nodes) {
      if (n == null) continue;
      navNodes.add(n);
      navNodeMap.put(n.id, n);
    }

    for (NavNode n : navNodes) {
      List<String> keep = new ArrayList<>();
      if (n.links != null) {
        for (String id : n.links) {
          if (navNodeMap.containsKey(id)) keep.add(id);
        }
      }
      n.links = keep;
    }
  }

  public static void setRobotDimensions(double diagonalCrossMeters) {
    robotInflationRadius = Math.max(0.0, diagonalCrossMeters / 2.0);
  }

  public static void addObstacle(Pose2d cornerA, Pose2d cornerB) {
    if (cornerA == null || cornerB == null) return;
    obstacles.add(new RectObstacle(cornerA.getTranslation(), cornerB.getTranslation()));
  }

  public static void clearObstacles() {
    obstacles.clear();
    inflated.clear();
  }

  public static List<Waypoint> generatePath(Pose2d start, Pose2d end) {
    if (!runBefore) {
      lastPose = end;
      runBefore = true;
    } else {
      start = lastPose;
      lastPose = end;
    }

    return generatePathInternal(start, new Pose2d[] {end});
  }

  public static List<Waypoint> generatePath(Pose2d start, Translation2d end) {
    if (!runBefore) {
      lastPose = new Pose2d(end, new Rotation2d());
      runBefore = true;
    } else {
      start = lastPose;
      lastPose = new Pose2d(end, new Rotation2d());
    }

    return generatePathInternal(start, new Pose2d[] {new Pose2d(end, new Rotation2d())});
  }

  public static List<Waypoint> generatePath(Pose2d start, Pose2d... midAndEnd) {
    if (!runBefore) {
      lastPose = midAndEnd[midAndEnd.length - 1];
      runBefore = true;
    } else {
      start = lastPose;
      lastPose = midAndEnd[midAndEnd.length - 1];
    }

    if (midAndEnd == null || midAndEnd.length == 0)
      throw new IllegalArgumentException("Must provide at least end Pose2d");
    return generatePathInternal(start, midAndEnd);
  }

  public static List<Waypoint> generatePath(Pose2d start, Translation2d... midAndEnd) {
    if (!runBefore) {
      lastPose = new Pose2d(midAndEnd[midAndEnd.length - 1], new Rotation2d());
      runBefore = true;
    } else {
      start = lastPose;
      lastPose = new Pose2d(midAndEnd[midAndEnd.length - 1], new Rotation2d());
    }

    Pose2d[] poses = new Pose2d[midAndEnd.length];
    int i = 0;
    for (Translation2d translation : midAndEnd) {
      poses[i] = new Pose2d(translation, new Rotation2d());
      i++;
    }

    if (midAndEnd == null || midAndEnd.length == 0)
      throw new IllegalArgumentException("Must provide at least end Pose2d");
    return generatePathInternal(start, poses);
  }

  private static List<Waypoint> generatePathInternal(Pose2d start, Pose2d[] midAndEnd) {
    if (globalConstraints == null)
      throw new IllegalStateException("Call configureConstraints(...) before generatePath");

    if (globalConstraints == null)
      throw new IllegalStateException("Call configureConstraints(...) before generatePath");

    List<Pose2d> requested = new ArrayList<>(1 + midAndEnd.length);
    requested.add(start);
    Collections.addAll(requested, midAndEnd);

    recomputeInflatedObstacles();

    // 1) Connect requested anchors using node graph with explicit links
    List<Pose2d> anchors = new ArrayList<>();
    List<Boolean> isRequestedAnchor = new ArrayList<>();

    for (int i = 0; i < requested.size() - 1; i++) {
      Pose2d s = requested.get(i);
      Pose2d g = requested.get(i + 1);
      List<Translation2d> nodePath = runAStarWithLinks(s.getTranslation(), g.getTranslation());
      for (int j = 0; j < nodePath.size(); j++) {
        Translation2d t = nodePath.get(j);
        Pose2d p;
        boolean requestedFlag = false;
        if (i == 0 && j == 0) {
          p = s;
          requestedFlag = true;
        } else if (j == nodePath.size() - 1) {
          p = g;
          requestedFlag = true;
        } else {
          p = new Pose2d(t, new Rotation2d());
          requestedFlag = false;
        }
        if (anchors.isEmpty() || !posesEqual(anchors.get(anchors.size() - 1), p)) {
          anchors.add(p);
          isRequestedAnchor.add(requestedFlag);
        }
      }
    }
    Pose2d finalRequested = requested.get(requested.size() - 1);
    if (anchors.isEmpty() || !posesEqual(anchors.get(anchors.size() - 1), finalRequested)) {
      anchors.add(finalRequested);
      isRequestedAnchor.add(true);
    }

    // 2) initial headings (point toward next) — if anchor was requested, use its rotation as the
    // initial heading target.
    final int n = anchors.size();
    Rotation2d[] headings = new Rotation2d[n];
    for (int i = 0; i < n; i++) {
      if (i < n - 1) {
        if (isRequestedAnchor.size() > i && isRequestedAnchor.get(i)) {
          headings[i] = anchors.get(i).getRotation();
        } else {
          Translation2d delta =
              anchors.get(i + 1).getTranslation().minus(anchors.get(i).getTranslation());
          headings[i] =
              (delta.getNorm() < 1e-9)
                  ? anchors.get(i).getRotation()
                  : new Rotation2d(delta.getX(), delta.getY());
        }
      } else {
        headings[i] = anchors.get(i).getRotation();
      }
    }

    Map<String, Evaluation> cache = new HashMap<>(1024);
    int maxPasses = 3;
    for (int pass = 0; pass < maxPasses; pass++) {
      boolean improved = false;
      double range = ANGLE_RANGE_DEG;
      double step = (pass == 0) ? COARSE_STEP_DEG : FINE_STEP_DEG;
      double refineRange = (pass == 0) ? range : Math.min(range, COARSE_STEP_DEG);

      for (int idx = 0; idx < n; idx++) {
        Rotation2d base = headings[idx];
        Rotation2d best = base;
        double bestCost = Double.POSITIVE_INFINITY;

        for (double off = -refineRange; off <= refineRange; off += step) {
          Rotation2d cand = base.plus(Rotation2d.fromDegrees(off));
          headings[idx] = cand;

          String key = headingsKey(anchors, headings);
          Evaluation ev = cache.get(key);
          if (ev == null) {
            if (controlsIntersectAnyObstacle(anchors, headings)) {
              ev = new Evaluation(true, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
            } else {
              List<Waypoint> wps = buildWaypoints(anchors, headings);
              GoalEndState end = guessGoalEndStateFromWaypoints(wps);
              PathPlannerPath path;
              try {
                path = new PathPlannerPath(wps, globalConstraints, null, end);
              } catch (Throwable ex) {
                ev = new Evaluation(true, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
                cache.put(key, ev);
                continue;
              }
              PathPlannerTrajectory traj;
              try {
                traj =
                    new PathPlannerTrajectory(
                        path, new ChassisSpeeds(), new Rotation2d(), globalConfig);
              } catch (Throwable ex) {
                try {
                  traj =
                      new PathPlannerTrajectory(
                          path, new ChassisSpeeds(), new Rotation2d(), globalConfig);
                } catch (Throwable ex2) {
                  ev = new Evaluation(true, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
                  cache.put(key, ev);
                  continue;
                }
              }
              ev = evaluateTrajectory(traj);
            }
            cache.put(key, ev);
          }

          if (!ev.collision) {
            double cost = ev.length + CURVATURE_WEIGHT * ev.maxCurvature;
            if (cost < bestCost) {
              bestCost = cost;
              best = cand;
            }
          }
        }

        if (!best.equals(headings[idx])) {
          headings[idx] = best;
          improved = true;
        } else headings[idx] = best;
      }

      if (!improved) break;
    }

    List<Waypoint> finalWps = buildWaypoints(anchors, headings);
    GoalEndState finalEnd = guessGoalEndStateFromWaypoints(finalWps);
    PathPlannerPath finalPath = new PathPlannerPath(finalWps, globalConstraints, null, finalEnd);

    PathPlannerTrajectory finalTraj;
    try {
      finalTraj =
          new PathPlannerTrajectory(finalPath, new ChassisSpeeds(), new Rotation2d(), globalConfig);
    } catch (Throwable ex) {
      finalTraj =
          new PathPlannerTrajectory(finalPath, new ChassisSpeeds(), new Rotation2d(), globalConfig);
    }

    return finalPath.getWaypoints();
  }

  // waypoint builder
  private static List<Waypoint> buildWaypoints(List<Pose2d> anchors, Rotation2d[] headings) {
    List<Waypoint> out = new ArrayList<>(anchors.size());
    for (int i = 0; i < anchors.size(); i++) {
      Translation2d prev = (i > 0) ? anchors.get(i - 1).getTranslation() : null;
      Translation2d next = (i < anchors.size() - 1) ? anchors.get(i + 1).getTranslation() : null;
      out.add(autoControlPointsManual(anchors.get(i).getTranslation(), headings[i], prev, next));
    }
    return out;
  }

  private static Waypoint autoControlPointsManual(
      Translation2d anchor,
      Rotation2d heading,
      Translation2d prevAnchor,
      Translation2d nextAnchor) {
    Translation2d prevControl = null;
    Translation2d nextControl = null;
    double hx = heading.getCos();
    double hy = heading.getSin();
    if (prevAnchor != null) {
      double d = Math.max(1e-6, anchor.getDistance(prevAnchor) * AUTO_CONTROL_DISTANCE_FACTOR);
      prevControl = anchor.minus(new Translation2d(d * hx, d * hy));
    }
    if (nextAnchor != null) {
      double d = Math.max(1e-6, anchor.getDistance(nextAnchor) * AUTO_CONTROL_DISTANCE_FACTOR);
      nextControl = anchor.plus(new Translation2d(d * hx, d * hy));
    }
    return new Waypoint(prevControl, anchor, nextControl);
  }

  private static boolean controlsIntersectAnyObstacle(List<Pose2d> anchors, Rotation2d[] headings) {
    int n = anchors.size();
    for (int i = 0; i < n; i++) {
      Translation2d anchor = anchors.get(i).getTranslation();
      Translation2d prev = (i > 0) ? anchors.get(i - 1).getTranslation() : null;
      Translation2d next = (i < n - 1) ? anchors.get(i + 1).getTranslation() : null;
      Rotation2d h = headings[i];

      if (prev != null) {
        double d = Math.max(1e-6, anchor.getDistance(prev) * AUTO_CONTROL_DISTANCE_FACTOR);
        Translation2d prevControl = anchor.minus(new Translation2d(d * h.getCos(), d * h.getSin()));
        if (segmentIntersectsAnyInflatedObstacle(prevControl, anchor)) return true;
        if (segmentIntersectsAnyInflatedObstacle(prev, prevControl)) return true;
      }
      if (next != null) {
        double d = Math.max(1e-6, anchor.getDistance(next) * AUTO_CONTROL_DISTANCE_FACTOR);
        Translation2d nextControl = anchor.plus(new Translation2d(d * h.getCos(), d * h.getSin()));
        if (segmentIntersectsAnyInflatedObstacle(anchor, nextControl)) return true;
        if (segmentIntersectsAnyInflatedObstacle(nextControl, next)) return true;
      }
      if (next != null && segmentIntersectsAnyInflatedObstacle(anchor, next)) return true;
    }
    return false;
  }

  private static boolean segmentIntersectsAnyInflatedObstacle(Translation2d a, Translation2d b) {
    for (RectObstacle r : inflated) if (r.intersectsSegmentAnalytic(a, b)) return true;
    return false;
  }

  // trajectory evaluation
  private static final class Evaluation {
    final boolean collision;
    final double maxCurvature;
    final double length;

    Evaluation(boolean collision, double maxCurvature, double length) {
      this.collision = collision;
      this.maxCurvature = maxCurvature;
      this.length = length;
    }
  }

  private static Evaluation evaluateTrajectory(PathPlannerTrajectory traj) {
    if (traj == null)
      return new Evaluation(true, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    double totalT = traj.getTotalTimeSeconds();
    double maxCurv = 0.0;
    double len = 0.0;
    boolean coll = false;
    PathPlannerTrajectoryState prev = null;

    for (double t = 0.0; t <= totalT + 1e-9; t += SAMPLE_DT) {
      PathPlannerTrajectoryState s;
      try {
        s = traj.sample(t);
      } catch (Throwable ex) {
        return new Evaluation(true, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      }
      Pose2d pose = s.pose;
      Translation2d pos = pose.getTranslation();
      if (pointInInflatedObstacle(pos)) {
        coll = true;
        break;
      }

      if (prev != null) {
        Translation2d prevPos = prev.pose.getTranslation();
        double d = pos.getDistance(prevPos);
        len += d;
        double prevH = prev.heading.getRadians();
        double curH = s.heading.getRadians();
        double dtheta = normalizeAngle(curH - prevH);
        if (d > CURV_EPS_DIST) {
          double curv = Math.abs(dtheta / d);
          if (!Double.isNaN(curv) && !Double.isInfinite(curv)) maxCurv = Math.max(maxCurv, curv);
        }
      }
      prev = s;
    }

    return new Evaluation(coll, maxCurv, len);
  }

  private static double normalizeAngle(double a) {
    while (a <= -Math.PI) a += 2 * Math.PI;
    while (a > Math.PI) a -= 2 * Math.PI;
    return a;
  }

  // DOESN'T WORK I MESSED UP
  public static GoalEndState guessGoalEndStateFromWaypoints(List<Waypoint> waypoints) {
    if (waypoints == null || waypoints.isEmpty()) return new GoalEndState(0.0, new Rotation2d());
    try {
      Waypoint last = waypoints.get(waypoints.size() - 1);
      Translation2d anchor = extractAnchor(last);

      Translation2d nextControl =
          extractControlPoint(last, "getNextControl", "nextControl", "next");
      Translation2d prevControl =
          extractControlPoint(last, "getPrevControl", "prevControl", "prev");
      if (anchor != null) {
        if (nextControl != null) return new GoalEndState(0.0, headingFrom(anchor, nextControl));
        if (prevControl != null) return new GoalEndState(0.0, headingFrom(prevControl, anchor));
      }

      if (waypoints.size() >= 2) {
        Translation2d a1 = extractAnchor(waypoints.get(waypoints.size() - 2));
        Translation2d a2 = extractAnchor(waypoints.get(waypoints.size() - 1));
        if (a1 != null && a2 != null) return new GoalEndState(0.0, headingFrom(a1, a2));
      }
    } catch (Throwable ignored) {
    }
    return new GoalEndState(0.0, new Rotation2d());
  }

  // helpers ig?

  private static Rotation2d headingFrom(Translation2d from, Translation2d to) {
    double dx = to.getX() - from.getX();
    double dy = to.getY() - from.getY();
    if (Math.hypot(dx, dy) < 1e-9) return new Rotation2d();
    return new Rotation2d(dx, dy);
  }

  private static Translation2d extractAnchor(Waypoint wp) {
    if (wp == null) return null;
    try {
      java.lang.reflect.Method m = wp.getClass().getMethod("getAnchor");
      Object res = m.invoke(wp);
      if (res instanceof Translation2d) return (Translation2d) res;
    } catch (Throwable ignored) {
    }
    try {
      java.lang.reflect.Field f = wp.getClass().getField("anchor");
      Object res = f.get(wp);
      if (res instanceof Translation2d) return (Translation2d) res;
    } catch (Throwable ignored) {
    }
    return null;
  }

  private static Translation2d extractControlPoint(Waypoint wp, String... names) {
    if (wp == null) return null;
    for (String name : names) {
      try {
        java.lang.reflect.Method m = wp.getClass().getMethod(name);
        Object res = m.invoke(wp);
        if (res instanceof Translation2d) return (Translation2d) res;
      } catch (Throwable ignored) {
      }
      try {
        java.lang.reflect.Field f = wp.getClass().getField(name);
        Object res = f.get(wp);
        if (res instanceof Translation2d) return (Translation2d) res;
      } catch (Throwable ignored) {
      }
    }
    return null;
  }

  // my astar but its going to fail watch
  private static List<Translation2d> runAStarWithLinks(Translation2d start, Translation2d goal) {
    // if no nav nodes, return straight
    if (navNodes.isEmpty()) return List.of(start, goal);

    // build graph nodes for each NavNode
    Map<String, GraphNode> graph = new HashMap<>();
    for (NavNode nn : navNodes) graph.put(nn.id, new GraphNode(nn));

    // THIS BETTER OBEY MY LINKS
    for (NavNode nn : navNodes) {
      GraphNode a = graph.get(nn.id);
      if (nn.links == null) continue;
      for (String linkId : nn.links) {
        GraphNode b = graph.get(linkId);
        if (b == null) continue;
        a.neighbors.add(b);
      }
    }

    List<GraphNode> nodeList = new ArrayList<>(graph.values());

    // nearest navnode to end and start
    GraphNode nearestToStart = null;
    GraphNode nearestToGoal = null;
    double bestStartDist = Double.POSITIVE_INFINITY;
    double bestGoalDist = Double.POSITIVE_INFINITY;
    for (GraphNode node : nodeList) {
      Translation2d t = node.nav.pose.getTranslation();
      double ds = t.getDistance(start);
      if (ds < bestStartDist) {
        bestStartDist = ds;
        nearestToStart = node;
      }
      double dg = t.getDistance(goal);
      if (dg < bestGoalDist) {
        bestGoalDist = dg;
        nearestToGoal = node;
      }
    }

    // print chosen nearest nodes for debugging, super useful to find out all my mistakes
    try {
      String ns = (nearestToStart != null) ? nearestToStart.nav.id : "NULL";
      String ng = (nearestToGoal != null) ? nearestToGoal.nav.id : "NULL";
      System.out.println("[NodePathGenerator] A* nearestStart: " + ns + " nearestGoal: " + ng);
    } catch (Throwable ignored) {
    }

    java.util.function.BiFunction<GraphNode, GraphNode, List<GraphNode>> runAstarBetween =
        (sNode, gNode) -> {
          if (sNode == null || gNode == null) return null;
          // reset state
          for (GraphNode gn : graph.values()) {
            gn.g = Double.POSITIVE_INFINITY;
            gn.f = Double.POSITIVE_INFINITY;
            gn.parent = null;
          }
          PriorityQueue<GraphNode> open = new PriorityQueue<>(Comparator.comparingDouble(n -> n.f));
          Set<GraphNode> closed = new HashSet<>();

          sNode.g = 0.0;
          sNode.f = distance(sNode.nav.pose.getTranslation(), gNode.nav.pose.getTranslation());
          open.add(sNode);

          while (!open.isEmpty()) {
            GraphNode cur = open.poll();
            if (cur == gNode) {
              // reconstruct path
              List<GraphNode> path = new ArrayList<>();
              GraphNode p = cur;
              while (p != null) {
                path.add(p);
                p = p.parent;
              }
              Collections.reverse(path);
              return path;
            }
            closed.add(cur);

            for (GraphNode nb : cur.neighbors) {
              if (closed.contains(nb)) continue;
              double tentative =
                  cur.g + cur.nav.pose.getTranslation().getDistance(nb.nav.pose.getTranslation());
              if (tentative < nb.g) {
                nb.parent = cur;
                nb.g = tentative;
                nb.f =
                    tentative
                        + distance(nb.nav.pose.getTranslation(), gNode.nav.pose.getTranslation());
                if (!open.contains(nb)) open.add(nb);
              }
            }
          }
          return null; // no path along declared linkages
        };

    // 1) Try the straightforward case: A* from nearestToStart -> nearestToGoal following declared
    // links
    List<GraphNode> path = runAstarBetween.apply(nearestToStart, nearestToGoal);

    // 2) If no path found, attempt from nearestToStart to any node reachable that is
    // as-close-as-possible to goal:
    if (path == null) {
      double bestGoalDistAfter = Double.POSITIVE_INFINITY;
      List<GraphNode> bestCandidate = null;
      for (GraphNode candGoal : nodeList) {
        List<GraphNode> p = runAstarBetween.apply(nearestToStart, candGoal);
        if (p == null) continue;
        double lastDist = p.get(p.size() - 1).nav.pose.getTranslation().getDistance(goal);
        if (lastDist < bestGoalDistAfter) {
          bestGoalDistAfter = lastDist;
          bestCandidate = p;
        }
      }
      path = bestCandidate;
    }

    // 3) If still no path, try all possible start nodes (explicit linkage-only search) and pick the
    // one with minimal total cost
    if (path == null) {
      double bestTotal = Double.POSITIVE_INFINITY;
      List<GraphNode> bestPath = null;
      for (GraphNode sNode : nodeList) {
        for (GraphNode gNode : nodeList) {
          List<GraphNode> p = runAstarBetween.apply(sNode, gNode);
          if (p == null || p.isEmpty()) continue;
          double pathLen = 0.0;
          for (int i = 0; i < p.size() - 1; i++) {
            pathLen +=
                p.get(i)
                    .nav
                    .pose
                    .getTranslation()
                    .getDistance(p.get(i + 1).nav.pose.getTranslation());
          }
          double total =
              start.getDistance(p.get(0).nav.pose.getTranslation())
                  + pathLen
                  + p.get(p.size() - 1).nav.pose.getTranslation().getDistance(goal);
          if (total < bestTotal) {
            bestTotal = total;
            bestPath = p;
          }
        }
      }
      path = bestPath;
    }

    if (path == null) {
      System.out.println("No path found - CSPPathing");
      List<Translation2d> direct = new ArrayList<>();
      direct.add(start);
      direct.add(goal);
      return direct;
    }

    // Convert GraphNode path into translations
    List<Translation2d> result = new ArrayList<>();
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < path.size(); i++) {
      GraphNode gn = path.get(i);
      result.add(gn.nav.pose.getTranslation());
      if (i > 0) sb.append(" -> ");
      sb.append(gn.nav.id);
    }

    // Print the chosen node path
    System.out.println("A* node path: " + sb.toString());

    return result;
  }

  private static double distance(Translation2d a, Translation2d b) {
    return a.getDistance(b);
  }

  public static boolean robotOnPath(Pose2d robotPose) {
    return (distance(robotPose.getTranslation(), lastPose.getTranslation()) > PATH_ERROR_METERS);
  }

  public static void setLastPose(Pose2d pose) {
    lastPose = pose;
  }

  // inflate obstacles
  private static void recomputeInflatedObstacles() {
    inflated.clear();
    for (RectObstacle r : obstacles) inflated.add(r.inflate(robotInflationRadius));
  }

  private static boolean pointInInflatedObstacle(Translation2d p) {
    for (RectObstacle r : inflated) if (r.contains(p)) return true;
    return false;
  }

  // other helpers and classes
  public static final class NavNode {
    public final String id;
    public final Pose2d pose;
    public List<String> links;

    public NavNode(String id, Pose2d pose, List<String> links) {
      this.id = Objects.requireNonNull(id);
      this.pose = Objects.requireNonNull(pose);
      this.links = (links == null) ? new ArrayList<>() : new ArrayList<>(links);
    }
  }

  private static final class GraphNode {
    final NavNode nav;
    double g = Double.POSITIVE_INFINITY, f = Double.POSITIVE_INFINITY;
    GraphNode parent = null;
    final List<GraphNode> neighbors = new ArrayList<>();

    GraphNode(NavNode nav) {
      this.nav = nav;
    }
  }

  private static final class RectObstacle {
    final double minX, maxX, minY, maxY;

    RectObstacle(Translation2d a, Translation2d b) {
      this.minX = Math.min(a.getX(), b.getX());
      this.maxX = Math.max(a.getX(), b.getX());
      this.minY = Math.min(a.getY(), b.getY());
      this.maxY = Math.max(a.getY(), b.getY());
    }

    RectObstacle inflate(double r) {
      return new RectObstacle(
          new Translation2d(minX - r, minY - r), new Translation2d(maxX + r, maxY + r));
    }

    boolean contains(Translation2d p) {
      return p.getX() >= minX && p.getX() <= maxX && p.getY() >= minY && p.getY() <= maxY;
    }

    boolean intersectsSegmentAnalytic(Translation2d a, Translation2d b) {
      if (contains(a) || contains(b)) return true;
      Translation2d r00 = new Translation2d(minX, minY);
      Translation2d r10 = new Translation2d(maxX, minY);
      Translation2d r11 = new Translation2d(maxX, maxY);
      Translation2d r01 = new Translation2d(minX, maxY);
      if (segmentsIntersect(a, b, r00, r10)) return true;
      if (segmentsIntersect(a, b, r10, r11)) return true;
      if (segmentsIntersect(a, b, r11, r01)) return true;
      if (segmentsIntersect(a, b, r01, r00)) return true;
      return false;
    }

    private static boolean segmentsIntersect(
        Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
      double x1 = p1.getX(), y1 = p1.getY(), x2 = p2.getX(), y2 = p2.getY();
      double x3 = q1.getX(), y3 = q1.getY(), x4 = q2.getX(), y4 = q2.getY();
      double o1 = orient(x1, y1, x2, y2, x3, y3);
      double o2 = orient(x1, y1, x2, y2, x4, y4);
      double o3 = orient(x3, y3, x4, y4, x1, y1);
      double o4 = orient(x3, y3, x4, y4, x2, y2);
      if (o1 * o2 < 0 && o3 * o4 < 0) return true;
      if (Math.abs(o1) < 1e-9 && onSegment(x1, y1, x3, y3, x2, y2)) return true;
      if (Math.abs(o2) < 1e-9 && onSegment(x1, y1, x4, y4, x2, y2)) return true;
      if (Math.abs(o3) < 1e-9 && onSegment(x3, y3, x1, y1, x4, y4)) return true;
      if (Math.abs(o4) < 1e-9 && onSegment(x3, y3, x2, y2, x4, y4)) return true;
      return false;
    }

    private static double orient(double ax, double ay, double bx, double by, double cx, double cy) {
      return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    }

    private static boolean onSegment(
        double ax, double ay, double bx, double by, double cx, double cy) {
      return bx >= Math.min(ax, cx) - 1e-9
          && bx <= Math.max(ax, cx) + 1e-9
          && by >= Math.min(ay, cy) - 1e-9
          && by <= Math.max(ay, cy) + 1e-9;
    }
  }

  private static String headingsKey(List<Pose2d> anchors, Rotation2d[] headings) {
    StringBuilder sb = new StringBuilder(anchors.size() * 16);
    for (int i = 0; i < anchors.size(); i++) {
      Pose2d p = anchors.get(i);
      sb.append(String.format(Locale.ROOT, "%.3f,%.3f:", p.getX(), p.getY()));
      sb.append(String.format(Locale.ROOT, "%.3f;", headings[i].getDegrees()));
    }
    return sb.toString();
  }

  private static boolean posesEqual(Pose2d a, Pose2d b) {
    if (a == null || b == null) return false;
    return Math.abs(a.getX() - b.getX()) < 1e-6 && Math.abs(a.getY() - b.getY()) < 1e-6;
  }
}
