// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.google.common.collect.Sets;
import com.google.common.graph.GraphBuilder;
import com.google.common.graph.MutableGraph;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.ExtensionKinematics.ExtensionState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class ExtensionPathing {
  public static final MutableGraph<ExtensionState> graph =
      GraphBuilder.undirected().allowsSelfLoops(true).build();
  // TODO make this cache distances so we can do partial caches
  private static final Map<Pair<ExtensionState, ExtensionState>, List<ExtensionState>> cache =
      new HashMap<>();

  static {
    final var hp =
        new ExtensionState(
            ElevatorSubsystem.HP_EXTENSION_METERS,
            ShoulderSubsystem.SHOULDER_HP_POS,
            WristSubsystem.WRIST_HP_POS);
    graph.addNode(hp);
    final var tucked =
        new ExtensionState(
            0.0,
            ShoulderSubsystem.SHOULDER_TUCKED_CLEARANCE_POS,
            WristSubsystem.WRIST_TUCKED_CLEARANCE_POS);
    graph.addNode(tucked);
    graph.putEdge(hp, tucked);
    final var l2Tucked =
        new ExtensionState(
            ExtensionKinematics.L2_EXTENSION.elevatorHeightMeters(),
            ShoulderSubsystem.SHOULDER_TUCKED_CLEARANCE_POS,
            WristSubsystem.WRIST_TUCKED_CLEARANCE_POS);
    graph.addNode(l2Tucked);
    graph.putEdge(tucked, l2Tucked);
    final var l3Tucked =
        new ExtensionState(
            ExtensionKinematics.L3_EXTENSION.elevatorHeightMeters(),
            ShoulderSubsystem.SHOULDER_TUCKED_CLEARANCE_POS,
            WristSubsystem.WRIST_TUCKED_CLEARANCE_POS);
    graph.addNode(l3Tucked);
    graph.putEdge(tucked, l3Tucked);
    final var l4Tucked =
        new ExtensionState(
            ExtensionKinematics.L4_EXTENSION.elevatorHeightMeters(),
            ShoulderSubsystem.SHOULDER_TUCKED_CLEARANCE_POS,
            WristSubsystem.WRIST_TUCKED_CLEARANCE_POS);
    graph.addNode(l4Tucked);
    graph.putEdge(tucked, l4Tucked);
    final var l4TuckedOut =
        new ExtensionState(
            ExtensionKinematics.L4_EXTENSION.elevatorHeightMeters(),
            Rotation2d.fromDegrees(30.0),
            Rotation2d.fromDegrees(120.0));
    graph.addNode(l4TuckedOut);
    graph.putEdge(l4Tucked, l4TuckedOut);

    final var betweenTucked =
        new ExtensionState(0.0, Rotation2d.fromDegrees(35.0), WristSubsystem.WRIST_CLEARANCE_POS);
    graph.addNode(betweenTucked);
    graph.putEdge(tucked, betweenTucked);

    final var untucked =
        new ExtensionState(
            0.0, ShoulderSubsystem.SHOULDER_CLEARANCE_POS, WristSubsystem.WRIST_CLEARANCE_POS);
    graph.addNode(untucked);
    graph.putEdge(betweenTucked, untucked);

    final var coralGround =
        new ExtensionState(
            ElevatorSubsystem.GROUND_EXTENSION_METERS,
            ShoulderSubsystem.SHOULDER_CORAL_GROUND_POS,
            Rotation2d.fromDegrees(45.0));
    graph.addNode(coralGround);
    graph.putEdge(untucked, coralGround);
  }

  private ExtensionPathing() {}

  public static ExtensionState getNearest(ExtensionState state) {
    return graph.nodes().stream()
        .min(
            (a, b) -> {
              var aFK = ExtensionKinematics.solveFK(a);
              var bFK = ExtensionKinematics.solveFK(b);
              var stateFK = ExtensionKinematics.solveFK(state);
              return (int)
                  (1000
                          * (aFK.getTranslation().getDistance(stateFK.getTranslation())
                              + Math.abs(
                                  aFK.getRotation().getRotations()
                                      - stateFK.getRotation().getRotations()))
                      - 1000
                          * (bFK.getTranslation().getDistance(stateFK.getTranslation())
                              + Math.abs(
                                  bFK.getRotation().getRotations()
                                      - stateFK.getRotation().getRotations())));
            })
        .get();
  }

  private static Pair<List<ExtensionState>, Double> search(
      ExtensionState current, ExtensionState target, Set<ExtensionState> visited) {
    if (current == target) {
      List<ExtensionState> result = new ArrayList<>();
      result.add(target);
      return Pair.of(result, 0.0);
    }

    final var edges = Sets.difference(graph.successors(current), visited);
    final List<Pair<List<ExtensionState>, Double>> result = new ArrayList<>();
    for (var edge : edges) {
      final var path = search(edge, target, Sets.union(visited, Set.of(current)));
      if (path != null) result.add(path);
    }
    if (result.size() == 0) return null;
    final var best = result.stream().min((a, b) -> (int) (a.getSecond() - b.getSecond())).get();
    final List<ExtensionState> newList = new ArrayList<>();
    newList.add(current);
    newList.addAll(best.getFirst());
    return Pair.of(
        newList,
        best.getSecond()
            + Math.abs(
                current.elevatorHeightMeters() - best.getFirst().get(0).elevatorHeightMeters()));
  }

  public static List<ExtensionState> getPath(ExtensionState current, ExtensionState target) {
    final var nearestCurrent = getNearest(current);
    final var nearestTarget = getNearest(target);
    final var path =
        cache.containsKey(Pair.of(nearestCurrent, nearestTarget))
            ? cache.get(Pair.of(nearestCurrent, nearestTarget))
            : search(nearestCurrent, nearestTarget, Set.of()).getFirst();
    path.add(target);
    cache.putIfAbsent(Pair.of(nearestCurrent, nearestTarget), path);
    return path;
  }
}
