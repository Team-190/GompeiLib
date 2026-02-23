package edu.wpi.team190.gompeilib.test;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.utility.Offset;
import org.junit.jupiter.api.Test;

class OffsetTests {

  @Test
  void testInRangeTrue() {
    Offset offset = new Offset(6.0, 2.0, 0.0, 12);
    assertTrue(offset.inRange());
    assertEquals(8.0, offset.getSetpoint(), 1e-9);
  }

  @Test
  void testInRangeFalseBelowMin() {
    Offset offset = new Offset(5.0, -10.0, 0.0, 12);
    assertFalse(offset.inRange());
    assertEquals(0, offset.getSetpoint(), 1e-9); // clamped to min
  }

  @Test
  void testInRangeFalseAboveMax() {
    Offset offset = new Offset(10.0, 10.0, 0.0, 12);
    assertFalse(offset.inRange());
    assertEquals(12, offset.getSetpoint(), 1e-9); // clamped to max
  }

  @Test
  void testExactlyAtMinBoundary() {
    Offset offset = new Offset(5.0, -5.0, 0.0, 12.0);
    assertTrue(offset.inRange());
    assertEquals(0.0, offset.getSetpoint(), 1e-9);
  }

  @Test
  void testExactlyAtMaxBoundary() {
    Offset offset = new Offset(7.0, 5.0, 0.0, 12.0);
    assertTrue(offset.inRange());
    assertEquals(12.0, offset.getSetpoint(), 1e-9);
  }

  @Test
  void testOffsetSetterChangesBehavior() {
    Offset offset = new Offset(10.0, 0.0, 0.0, 12.0);
    assertEquals(10.0, offset.getSetpoint(), 1e-9);

    offset.setOffset(6.0); // uses Lombok setter
    assertFalse(offset.inRange());
    assertEquals(12.0, offset.getSetpoint(), 1e-9);
  }

  @Test
  void testOffsetNegativeClamp() {
    Offset offset = new Offset(-6.0, -4.0, -12.0, 0.0);
    assertEquals(-10.0, offset.getSetpoint(), 1e-9);
  }

  // @Test
  // void testOffset
}
