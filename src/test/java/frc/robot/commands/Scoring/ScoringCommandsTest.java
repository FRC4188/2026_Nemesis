package frc.robot.commands.Scoring;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ScoringCommandsTest {
  @Test
  void preservesScoringFormulas() {
    assertEquals(1800.0, ScoringCommands.RPMRegress(1.5), 1e-9);
    assertEquals(38.0 * Math.pow(4.2 - 1.5, 2) + 1800.0, ScoringCommands.RPMRegress(4.2), 1e-9);
    assertEquals(
        Math.PI / 2.0 - Math.atan(7.0 / 3.0),
        ScoringCommands.inclineHueristic(3.0).getRadians(),
        1e-9);
  }
}
