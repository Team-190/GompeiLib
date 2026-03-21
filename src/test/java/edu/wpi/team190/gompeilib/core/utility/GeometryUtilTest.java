package edu.wpi.team190.gompeilib.core.utility;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.FieldSource;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class GeometryUtilTest {
  public static Translation2d[] TRANSLATION2D_CASES;
  public static Pair<Double, Double>[] XY_PAIR_CASES;
  public static Rotation2d[] ROTATION2D_CASES;
  public static Pose2d[] POSE2D_CASES;

  static {
    TRANSLATION2D_CASES =
        new Translation2d[] {
          Translation2d.kZero, new Translation2d(190.0, 190.0), new Translation2d(42.0, 42.0)
        };
    XY_PAIR_CASES =
        new Pair[] {new Pair<>(0.0, 0.0), new Pair<>(190.0, 190.0), new Pair<>(42.0, 42.0)};
    ROTATION2D_CASES =
        new Rotation2d[] {
          Rotation2d.kZero, Rotation2d.fromDegrees(190.0), Rotation2d.fromDegrees(42.0)
        };
    POSE2D_CASES =
        new Pose2d[] {
          Pose2d.kZero,
          new Pose2d(190.0, 190.0, Rotation2d.fromDegrees(190.0)),
          new Pose2d(42.0, 42.0, Rotation2d.fromDegrees(42.0))
        };
  }

  @Test
  @Order(1)
  void testConstruction() {
    new GeometryUtil();
  }

  @ParameterizedTest
  @FieldSource("TRANSLATION2D_CASES")
  @Order(2)
  void testToTransform2dGivenTranslation(Translation2d translation) {
    assertEquals(
        GeometryUtil.toTransform2d(translation),
        new Transform2d(translation.getX(), translation.getY(), new Rotation2d()));
  }

  @ParameterizedTest
  @FieldSource("XY_PAIR_CASES")
  @Order(3)
  void testToTransform2dGivenDoubles(Pair<Double, Double> xyPair) {
    double x = xyPair.getFirst();
    double y = xyPair.getSecond();

    assertEquals(GeometryUtil.toTransform2d(x, y), new Transform2d(x, y, new Rotation2d()));
  }

  @ParameterizedTest
  @FieldSource("ROTATION2D_CASES")
  @Order(4)
  void testToTransform2dGivenRotation2d(Rotation2d rotation) {
    assertEquals(
        GeometryUtil.toTransform2d(rotation), new Transform2d(new Translation2d(), rotation));
  }

  @ParameterizedTest
  @FieldSource("POSE2D_CASES")
  @Order(5)
  void testToTransform2dGivenPose2d(Pose2d pose) {
    assertEquals(
        GeometryUtil.toTransform2d(pose),
        new Transform2d(pose.getX(), pose.getY(), pose.getRotation()));
  }

  @Test
  @Order(6)
  void testPose2dIsZero() {
    assertTrue(GeometryUtil.isZero(Pose2d.kZero));
  }

  @Test
  @Order(7)
  void testPose2dThetaIsNotZero() {
    Pose2d pose2d = new Pose2d(0.0, 0.0, Rotation2d.kPi);
    assertFalse(GeometryUtil.isZero(pose2d));
  }

  @Test
  @Order(8)
  void testPose2dYIsNotZero() {
    Pose2d pose2d = new Pose2d(0.0, 42.0, Rotation2d.kZero);
    assertFalse(GeometryUtil.isZero(pose2d));
  }

  @Test
  @Order(9)
  void testPose2dThetaIsZero() {
    Pose2d pose2d = new Pose2d(42.0, 0, Rotation2d.kZero);
    assertFalse(GeometryUtil.isZero(pose2d));
  }

  @Test
  @Order(10)
  void testAnyPose2dsAreZero() {
    Pose2d[] cases = new Pose2d[] {POSE2D_CASES[0], POSE2D_CASES[1], POSE2D_CASES[2]};
    assertTrue(GeometryUtil.isZero(cases));
  }

  @Test
  @Order(11)
  void testAnyPose2dsAreNotZero() {
    Pose2d[] cases = new Pose2d[] {POSE2D_CASES[1], POSE2D_CASES[2]};
    assertFalse(GeometryUtil.isZero(cases));
  }

  @Test
  @Order(12)
  void testEmptyPose2dsAreNotZero() {
    Pose2d[] cases = new Pose2d[] {};
    assertFalse(GeometryUtil.isZero(cases));
  }

  @Test
  @Order(13)
  void testTranslation2dIsZero() {
    assertTrue(GeometryUtil.isZero(Translation2d.kZero));
  }

  @Test
  @Order(14)
  void testTranslation2dXIsNotZero() {
    Translation2d translation2d = new Translation2d(42.0, 0.0);
    assertFalse(GeometryUtil.isZero(translation2d));
  }

  @Test
  @Order(15)
  void testTranslation2dYIsNotZero() {
    Translation2d translation2d = new Translation2d(0.0, 42.0);
    assertFalse(GeometryUtil.isZero(translation2d));
  }

  @Test
  @Order(16)
  void testRotation2dIsZero() {
    assertTrue(GeometryUtil.isZero(Rotation2d.kZero));
  }

  @Test
  @Order(17)
  void testRotation2dIsNotZero() {
    Rotation2d rotation2d = new Rotation2d(Math.PI);
    assertFalse(GeometryUtil.isZero(rotation2d));
  }

  @Test
  @Order(18)
  void testPose2dIsNotNaN() {
    assertFalse(GeometryUtil.isNaN(Pose2d.kZero));
  }

  @Test
  @Order(19)
  void testPose2dXIsNaN() {
    Pose2d pose2d = new Pose2d(Double.NaN, 0.0, Rotation2d.kZero);
    assertTrue(GeometryUtil.isNaN(pose2d));
  }

  @Test
  @Order(20)
  void testPose2dYIsNaN() {
    Pose2d pose2d = new Pose2d(0.0, Double.NaN, Rotation2d.kZero);
    assertTrue(GeometryUtil.isNaN(pose2d));
  }

  @Test
  @Order(21)
  void testPose2dThetaIsNaN() {
    Pose2d pose2d = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(Double.NaN));
    assertTrue(GeometryUtil.isNaN(pose2d));
  }

  @Test
  @Order(22)
  void testAnyPose2dsAreNaN() {
    Pose2d nanPose = new Pose2d(Double.NaN, 190.0, Rotation2d.fromDegrees(190.0));
    Pose2d[] cases = new Pose2d[] {nanPose, POSE2D_CASES[0], POSE2D_CASES[1], POSE2D_CASES[2]};
    assertTrue(GeometryUtil.isNaN(cases));
  }

  @Test
  @Order(23)
  void testAnyPose2dsAreNotNaN() {
    Pose2d[] cases = new Pose2d[] {POSE2D_CASES[0], POSE2D_CASES[1], POSE2D_CASES[2]};
    assertFalse(GeometryUtil.isNaN(cases));
  }

  @Test
  @Order(24)
  void testEmptyPose2dsAreNotNaN() {
    Pose2d[] cases = new Pose2d[] {};
    assertFalse(GeometryUtil.isNaN(cases));
  }
}
