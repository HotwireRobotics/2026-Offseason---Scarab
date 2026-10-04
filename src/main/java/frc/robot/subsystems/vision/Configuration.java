package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;

public class Configuration {
  /** Maximal localization distance. */
  public static final Distance maximum = Inches.of(185);
  /** Camera ambiquity along translation and rotation axes. */
  public static final Matrix<N3, N1> standardDeviation = VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY);
}
