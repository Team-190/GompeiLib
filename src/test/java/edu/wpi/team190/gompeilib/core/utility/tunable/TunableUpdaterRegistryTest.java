package edu.wpi.team190.gompeilib.core.utility.tunable;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.Constraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TunableUpdaterRegistryTest {

  @BeforeEach
  void setUp() throws Exception {
    GompeiLib.deinit();
    GompeiLib.init(RobotMode.SIM, true, 0.02);
  }

  @Test
  void periodicWithNoRegistrationsDoesNotThrow() {
    assertDoesNotThrow(TunableUpdaterRegistery::periodic);
  }

  @Test
  void registerGainsAndPeriodicInvokesConsumer() {
    Gains gains =
        Gains.fromDoubles()
            .withPrefix("test/gains")
            .withKP(1.0)
            .withKI(2.0)
            .withKD(3.0)
            .withKS(4.0)
            .withKV(5.0)
            .withKA(6.0)
            .withKG(7.0)
            .build();

    AtomicInteger callCount = new AtomicInteger();
    AtomicReference<Gains> updatedGains = new AtomicReference<>();

    TunableUpdaterRegistery.registerGains(
        gains,
        g -> {
          callCount.incrementAndGet();
          updatedGains.set(g);
        });

    TunableUpdaterRegistery.periodic();

    assertEquals(1, callCount.get());
    assertSame(gains, updatedGains.get());
  }

  @Test
  void registerGainsIgnoresDuplicateRegistration() {
    Gains gains =
        Gains.fromDoubles()
            .withPrefix("test/gains/duplicate")
            .withKP(1.0)
            .withKI(2.0)
            .withKD(3.0)
            .withKS(4.0)
            .withKV(5.0)
            .withKA(6.0)
            .withKG(7.0)
            .build();

    AtomicInteger firstConsumerCalls = new AtomicInteger();
    AtomicInteger secondConsumerCalls = new AtomicInteger();

    TunableUpdaterRegistery.registerGains(gains, g -> firstConsumerCalls.incrementAndGet());
    TunableUpdaterRegistery.registerGains(gains, g -> secondConsumerCalls.incrementAndGet());

    TunableUpdaterRegistery.periodic();

    assertEquals(1, firstConsumerCalls.get());
    assertEquals(0, secondConsumerCalls.get());
  }

  @Test
  void registerConstraintsAndPeriodicInvokesConsumer() {
    LinearConstraints constraints =
        LinearConstraints.fromMeasures()
            .withPrefix("test/constraints")
            .withGoalTolerance(Units.Meters.of(0.1))
            .withMaxVelocity(Units.MetersPerSecond.of(2.0))
            .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(3.0))
            .build();

    AtomicInteger callCount = new AtomicInteger();
    AtomicReference<Constraints<?>> updatedConstraints = new AtomicReference<>();

    TunableUpdaterRegistery.registerConstraints(
        constraints,
        c -> {
          callCount.incrementAndGet();
          updatedConstraints.set(c);
        });

    TunableUpdaterRegistery.periodic();

    assertEquals(1, callCount.get());
    assertSame(constraints, updatedConstraints.get());
  }

  @Test
  void registerConstraintsIgnoresDuplicateRegistration() {
    LinearConstraints constraints =
        LinearConstraints.fromMeasures()
            .withPrefix("test/constraints/duplicate")
            .withGoalTolerance(Units.Meters.of(0.2))
            .withMaxVelocity(Units.MetersPerSecond.of(4.0))
            .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(6.0))
            .build();

    AtomicInteger firstConsumerCalls = new AtomicInteger();
    AtomicInteger secondConsumerCalls = new AtomicInteger();

    TunableUpdaterRegistery.registerConstraints(
        constraints, c -> firstConsumerCalls.incrementAndGet());
    TunableUpdaterRegistery.registerConstraints(
        constraints, c -> secondConsumerCalls.incrementAndGet());

    TunableUpdaterRegistery.periodic();

    assertEquals(1, firstConsumerCalls.get());
    assertEquals(0, secondConsumerCalls.get());
  }

  @Test
  void registerNumberAndPeriodicInvokesConsumerWithValues() {
    LoggedTunableNumber[] numbers = {
      new LoggedTunableNumber("test/number/one", 1.25),
      new LoggedTunableNumber("test/number/two", 2.5)
    };

    AtomicInteger callCount = new AtomicInteger();
    AtomicReference<double[]> updatedValues = new AtomicReference<>();

    TunableUpdaterRegistery.registerNumber(
        numbers,
        values -> {
          callCount.incrementAndGet();
          updatedValues.set(values);
        });

    TunableUpdaterRegistery.periodic();

    assertEquals(1, callCount.get());
    assertArrayEquals(new double[] {1.25, 2.5}, updatedValues.get());
  }

  @Test
  void registerNumberIgnoresDuplicateRegistrationForSameArrayInstance() {
    LoggedTunableNumber[] numbers = {
      new LoggedTunableNumber("test/number/duplicate/one", 3.0),
      new LoggedTunableNumber("test/number/duplicate/two", 4.0)
    };

    AtomicInteger firstConsumerCalls = new AtomicInteger();
    AtomicInteger secondConsumerCalls = new AtomicInteger();

    TunableUpdaterRegistery.registerNumber(numbers, values -> firstConsumerCalls.incrementAndGet());
    TunableUpdaterRegistery.registerNumber(
        numbers, values -> secondConsumerCalls.incrementAndGet());

    TunableUpdaterRegistery.periodic();

    assertEquals(1, firstConsumerCalls.get());
    assertEquals(0, secondConsumerCalls.get());
  }

  @Test
  void registerMeasureAndPeriodicInvokesRunnable() {
    LoggedTunableMeasure<?>[] measures = {
      new LoggedTunableMeasure<>("test/measure/one", Units.Meters.of(1.0)),
      new LoggedTunableMeasure<>("test/measure/two", Units.MetersPerSecond.of(2.0))
    };

    AtomicInteger callCount = new AtomicInteger();

    TunableUpdaterRegistery.registerMeasure(measures, callCount::incrementAndGet);

    TunableUpdaterRegistery.periodic();

    assertEquals(1, callCount.get());
  }

  @Test
  void registerMeasureIgnoresDuplicateRegistrationForSameArrayInstance() {
    LoggedTunableMeasure<?>[] measures = {
      new LoggedTunableMeasure<>("test/measure/duplicate/one", Units.Meters.of(5.0)),
      new LoggedTunableMeasure<>("test/measure/duplicate/two", Units.MetersPerSecond.of(6.0))
    };

    AtomicInteger firstConsumerCalls = new AtomicInteger();
    AtomicInteger secondConsumerCalls = new AtomicInteger();

    TunableUpdaterRegistery.registerMeasure(measures, firstConsumerCalls::incrementAndGet);
    TunableUpdaterRegistery.registerMeasure(measures, secondConsumerCalls::incrementAndGet);

    TunableUpdaterRegistery.periodic();

    assertEquals(1, firstConsumerCalls.get());
    assertEquals(0, secondConsumerCalls.get());
  }

  @SuppressWarnings("unchecked")
  private void clearRegistry(String fieldName) throws Exception {
    Field field = TunableUpdaterRegistery.class.getDeclaredField(fieldName);
    field.setAccessible(true);
    ((HashMap<Object, Consumer<?>>) field.get(null)).clear();
  }
}
