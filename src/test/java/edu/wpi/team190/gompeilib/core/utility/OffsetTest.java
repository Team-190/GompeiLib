package edu.wpi.team190.gompeilib.core.utility;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import org.junit.jupiter.api.*;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class OffsetTest {
  @Test
  @Order(1)
  public void testApplyZero() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset<>(s);
    assertEquals(d, offset.apply(d));
  }

  @Test
  @Order(2)
  public void testApplyIncrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset<>(s);
    offset.increment();
    assertEquals(d.plus(s), offset.apply(d));
  }

  @Test
  @Order(3)
  public void testApplyDecrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset<>(s);
    offset.decrement();
    assertEquals(d.minus(s), offset.apply(d));
  }

  @Test
  @Order(4)
  public void testApplyOffsetRepeatedly() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset<>(s);
    offset.decrement();
    offset.decrement();
    offset.decrement();
    offset.decrement();
    offset.increment();
    offset.decrement();
    offset.increment();
    assertEquals(d.minus(s).minus(s).minus(s), offset.apply(d));
  }

  @Test
  @Order(5)
  public void testResetOffset() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Offset<DistanceUnit> offset = new Offset<>(s);
    offset.decrement();
    offset.reset();
    assertEquals(d, offset.apply(d));
  }

  @Test
  @Order(6)
  public void testSpecificIncrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Distance a = Inches.of(4);
    Offset<DistanceUnit> offset = new Offset<>(s);
    offset.increment(a);
    assertEquals(d.plus(a), offset.apply(d));
  }

  @Test
  @Order(7)
  public void testSpecificDecrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Distance a = Inches.of(4);
    Offset<DistanceUnit> offset = new Offset<>(s);
    offset.decrement(a);
    assertEquals(d.minus(a), offset.apply(d));
  }

  @Test
  @Order(8)
  public void testApplyGhostOffsetMaximum() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);
    Offset<DistanceUnit> offset = new Offset<>(s, min, max);
    offset.increment();
    offset.increment();
    offset.increment();
    assertEquals(d.plus(max), offset.apply(d));
  }

  @Test
  @Order(9)
  public void testApplyGhostOffsetMinimum() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);
    Offset<DistanceUnit> offset = new Offset<>(s, min, max);
    offset.decrement();
    offset.decrement();
    offset.decrement();
    assertEquals(d, offset.apply(d));
  }

  @Test
  @Order(10)
  public void testApplyGhostOffsetMaximumOverride() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);
    Offset<DistanceUnit> offset = new Offset<>(s, min, max);
    offset.increment(s);
    offset.increment(s);
    offset.increment(s);
    assertEquals(d.plus(max), offset.apply(d));
  }

  @Test
  @Order(11)
  public void testApplyGhostOffsetMinimumOverride() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);
    Offset<DistanceUnit> offset = new Offset<>(s, min, max);
    offset.decrement(s);
    offset.decrement(s);
    offset.decrement(s);
    assertEquals(d, offset.apply(d));
  }

  @Test
  @Order(12)
  public void testOffsetGetter() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);
    Offset<DistanceUnit> offset = new Offset<>(s, min, max);
    offset.increment(s);
    assertEquals(Meters.of(1.0), offset.getOffset());
  }
}
