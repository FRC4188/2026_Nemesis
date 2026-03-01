// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhoton implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhoton(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.frontConnected = camera.isConnected();
    inputs.backConnected = camera.isConnected();
    // This is configd in photon vision (use localhost while on robo wifi)
    // 0 is april tag, 1 is hopefully ml, anything onwards doesnt exist, yet.
    if (camera.getPipelineIndex() == 0) {
      // Read new camera observations
      Set<Short> tagIds = new HashSet<>();
      List<PoseObservation> poseObservations = new LinkedList<>();

      for (var result : camera.getAllUnreadResults()) {
        // Update latest target observation
        if (result.hasTargets()) {
          inputs.latestTargetObservation =
              new TargetObservation(
                  Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                  Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
        } else {
          inputs.latestTargetObservation =
              new TargetObservation(new Rotation2d(), new Rotation2d());
        }

        // Add pose observation
        if (result.multitagResult.isPresent()) { // Multitag result
          var multitagResult = result.multitagResult.get();

          // Calculate robot pose
          Transform3d fieldToCamera = multitagResult.estimatedPose.best;
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Calculate average tag distance
          double totalTagDistance = 0.0;
          for (var target : result.targets) {
            totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          }

          // Add tag IDs
          tagIds.addAll(multitagResult.fiducialIDsUsed);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  multitagResult.estimatedPose.ambiguity, // Ambiguity
                  multitagResult.fiducialIDsUsed.size(), // Tag count
                  totalTagDistance / result.targets.size(), // Average tag distance
                  PoseObservationType.PHOTONVISION)); // Observation type

        } else if (!result.targets.isEmpty()) { // Single tag result
          var target = result.targets.get(0);

          // Calculate robot pose
          var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
          if (tagPose.isPresent()) {
            Transform3d fieldToTarget =
                new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
            Transform3d cameraToTarget = target.bestCameraToTarget;
            Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            Pose3d robotPose =
                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            // Add tag ID
            tagIds.add((short) target.fiducialId);

            // Add observation
            poseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    robotPose, // 3D pose estimate
                    target.poseAmbiguity, // Ambiguity
                    1, // Tag count
                    cameraToTarget.getTranslation().getNorm(), // Average tag distance
                    PoseObservationType.PHOTONVISION)); // Observation type
          }
        }
      }
      // Save pose observations to inputs object
      inputs.poseObservations = new PoseObservation[poseObservations.size()];
      for (int i = 0; i < poseObservations.size(); i++) {
        inputs.poseObservations[i] = poseObservations.get(i);
      }

      // Save tag IDs to inputs objects
      inputs.tagIds = new int[tagIds.size()];
      int i = 0;
      for (int id : tagIds) {
        inputs.tagIds[i++] = id;
      }
    } else if (camera.getPipelineIndex() == 1) {
      // Init a list of all poses
      ArrayList<Translation2d> fuelposes = new ArrayList<Translation2d>();
      for (var result : camera.getAllUnreadResults()) {
        if (result.hasTargets()) {
          List<PhotonTrackedTarget> targets = result.getTargets();
          for (PhotonTrackedTarget target : targets) {
            Translation2d pose = getTargetTranslation(target);
            fuelposes.add(pose);
          }
        }
      }
    }
  }

  public Pose2d getGridResults(ArrayList<Translation2d> fuelposes, Pose3d robotPose) {
    double cellSize = 0.6096; // 24 inches cause intake is like that big that number in meters btw
    Translation2d translation = new Translation2d();
    ArrayList<int[]> gridData = new ArrayList<int[]>();
    for (int i = 0; i < fuelposes.size(); i++) {
      int CellX = (int) Math.floor(fuelposes.get(i).getX() / cellSize);
      int CellY = (int) Math.floor(fuelposes.get(i).getY() / cellSize);
      gridData.add(new int[] {CellX, CellY});
    }
    int maxCount = 0;

    for (int i = 0; i < gridData.size(); i++) {
      int count = 0;
      int x = gridData.get(i)[0];
      int y = gridData.get(i)[1];
      for (int j = 0; j < gridData.size(); j++) {
        if (gridData.get(j)[0] == x && gridData.get(j)[1] == y) {
          count++;
        }
      }
      if (count > maxCount) {
        maxCount = count;
        translation = new Translation2d((x + 0.5) * cellSize, (y + 0.5) * cellSize);
      }
    }
    double inchesToMeters = 0.0254;
    Transform3d poseShift =
        new Transform3d(translation.getX()*inchesToMeters , translation.getY()*inchesToMeters, 0.0, new Rotation3d());

    Pose3d newPose = robotPose.transformBy(poseShift);
    return newPose.toPose2d();
  }

  private Translation2d getTargetTranslation(PhotonTrackedTarget target) {

    double yawDeg = target.getYaw(); // degrees
    double pitchDeg = target.getPitch(); // degrees

    // pls dont explode
    // TODO: double check what our extrema are tmrw akhil this is arbitrary for know
    double x = Math.max(-20, Math.min(0, pitchDeg));

    // stuff me and zaki did forward and back (just use a ruler a long one)
    double forwardInches =
        0.699005 * Math.pow(x, 3) + 13.00874 * Math.pow(x, 2) + 86.31382 * x + 266.36636;

    double forward = Units.inchesToMeters(forwardInches);

    // Side distance
    // (We hope and pray this works)
    double side = forward / Math.tan(Math.toRadians(90 + yawDeg));

    return new Translation2d(forward, side);
  }

  public Pose2d getCluster(Pose3d robotPose) {
    ArrayList<Translation2d> fuelposes = new ArrayList<Translation2d>();
    if (camera.getPipelineIndex() == 1) {
      // Init a list of all poses
      for (var result : camera.getAllUnreadResults()) {
        if (result.hasTargets()) {
          List<PhotonTrackedTarget> targets = result.getTargets();
          for (PhotonTrackedTarget target : targets) {
            Translation2d pose = getTargetTranslation(target);
            fuelposes.add(pose);
          }
        }
      }
    }
    return getGridResults(fuelposes, robotPose);
  }
}
