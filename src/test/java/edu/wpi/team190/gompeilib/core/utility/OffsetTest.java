package edu.wpi.team190.gompeilib.core.utility;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import org.junit.jupiter.api.*;

class OffsetTests {
  @Test
  public void testApplyZero() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset(d, s);
    assertEquals(d, offset.getNewSetpoint());
  }

  @Test
  public void testApplyIncrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset(d, s);
    offset.increment();
    assertEquals(d.plus(s), offset.getNewSetpoint());
  }

  @Test
  public void testApplyDecrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset(d, s);
    offset.decrement();
    assertEquals(d.minus(s), offset.getNewSetpoint());
  }

  @Test
  public void testApplyOffsetRepeatedly() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset(d, s);
    offset.decrement();
    offset.decrement();
    offset.decrement();
    offset.decrement();
    offset.increment();
    offset.decrement();
    offset.increment();
    assertEquals(d.minus(s).minus(s).minus(s), offset.getNewSetpoint());
  }

  @Test
  public void testResetOffset() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset(d, s);
    offset.decrement();
    offset.reset();
    assertEquals(d, offset.getNewSetpoint());
  }

  @Test
  public void testSpecificIncrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Distance a = Inches.of(4);
    Offset<DistanceUnit> offset = new Offset(d, s);
    offset.increment(a);
    assertEquals(d.plus(a), offset.getNewSetpoint());
  }

  @Test
  public void testSpecificDecrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Distance a = Inches.of(4);
    Offset<DistanceUnit> offset = new Offset(d, s);
    offset.decrement(a);
    assertEquals(d.minus(a), offset.getNewSetpoint());
  }

  @Test
  public void testApplyGhostOffsetMaximum() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);
    Offset<DistanceUnit> offset = new Offset(d, s, min, max);
    offset.increment();
    offset.increment();
    offset.decrement();
    assertEquals(d, offset.getNewSetpoint());
  }

  @Test
  public void testApplyGhostOffsetMinimum() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);
    Offset<DistanceUnit> offset = new Offset(d, s, min, max);
    offset.decrement();
    offset.decrement();
    offset.increment();
    assertEquals(d, offset.getNewSetpoint());
  }

  @Test
  public void testApplyGhostOffsetMaximumOverride() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);
    Offset<DistanceUnit> offset = new Offset(d, s, min, max);
    offset.increment(s);
    offset.increment(s);
    offset.decrement(s);
    assertEquals(d, offset.getNewSetpoint());
  }

  @Test
  public void testApplyGhostOffsetMinimumOverride() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);
    Offset<DistanceUnit> offset = new Offset(d, s, min, max);
    offset.decrement(s);
    offset.decrement(s);
    offset.increment(s);
    assertEquals(d, offset.getNewSetpoint());
  }
}
