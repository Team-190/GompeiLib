package edu.wpi.team190.gompeilib.core.utility.tunable;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TunableUpdaterRegistryTest {

  @BeforeEach
  void setUp() {
    GompeiLib.deinit();
    GompeiLib.init(RobotMode.SIM, true, 0.02);
  }

  @Test
  void periodicWithNoRegistrationsDoesNothing() {
    assertDoesNotThrow(TunableUpdaterRegistry::periodic);
  }

  @Test
  void registerGainsInvokesConsumer() {
    Gains gains =
        Gains.fromDoubles()
            .withPrefix("test/gains")
            .withKP(1)
            .withKI(2)
            .withKD(3)
            .withKS(4)
            .withKV(5)
            .withKA(6)
            .withKG(7)
            .build();

    AtomicInteger calls = new AtomicInteger();

    TunableUpdaterRegistry.registerGains(gains, g -> calls.incrementAndGet());

    TunableUpdaterRegistry.periodic();

    assertEquals(1, calls.get());
  }

  @Test
  void registerGainsDuplicateIgnored() {
    Gains gains =
        Gains.fromDoubles()
            .withPrefix("test/gains/dup")
            .withKP(1)
            .withKI(2)
            .withKD(3)
            .withKS(4)
            .withKV(5)
            .withKA(6)
            .withKG(7)
            .build();

    AtomicInteger first = new AtomicInteger();
    AtomicInteger second = new AtomicInteger();

    TunableUpdaterRegistry.registerGains(gains, g -> first.incrementAndGet());
    TunableUpdaterRegistry.registerGains(gains, g -> second.incrementAndGet());

    TunableUpdaterRegistry.periodic();

    assertEquals(1, first.get());
    assertEquals(0, second.get());
  }

  @Test
  void registerConstraintsInvokesConsumer() {
    LinearConstraints constraints =
        LinearConstraints.fromMeasures()
            .withPrefix("test/constraints")
            .withGoalTolerance(Units.Meters.of(0.1))
            .withMaxVelocity(Units.MetersPerSecond.of(2))
            .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(3))
            .build();

    AtomicInteger calls = new AtomicInteger();

    TunableUpdaterRegistry.registerConstraints(constraints, c -> calls.incrementAndGet());

    TunableUpdaterRegistry.periodic();

    assertEquals(1, calls.get());
  }

  @Test
  void registerConstraintsDuplicateIgnored() {
    LinearConstraints constraints =
        LinearConstraints.fromMeasures()
            .withPrefix("test/constraints/dup")
            .withGoalTolerance(Units.Meters.of(0.2))
            .withMaxVelocity(Units.MetersPerSecond.of(3))
            .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(4))
            .build();

    AtomicInteger first = new AtomicInteger();
    AtomicInteger second = new AtomicInteger();

    TunableUpdaterRegistry.registerConstraints(constraints, c -> first.incrementAndGet());
    TunableUpdaterRegistry.registerConstraints(constraints, c -> second.incrementAndGet());

    TunableUpdaterRegistry.periodic();

    assertEquals(1, first.get());
    assertEquals(0, second.get());
  }

  @Test
  void registerNumberAcceptsArray() {
    LoggedTunableNumber[] nums = new LoggedTunableNumber[0];

    assertDoesNotThrow(() -> TunableUpdaterRegistry.registerNumber(nums, v -> {}));
  }

  @Test
  void registerNumberDuplicateIgnored() {
    LoggedTunableNumber[] nums = new LoggedTunableNumber[0];

    AtomicInteger first = new AtomicInteger();
    AtomicInteger second = new AtomicInteger();

    TunableUpdaterRegistry.registerNumber(nums, v -> first.incrementAndGet());
    TunableUpdaterRegistry.registerNumber(nums, v -> second.incrementAndGet());

    TunableUpdaterRegistry.periodic();

    assertEquals(1, first.get());
    assertEquals(0, second.get());
  }

  @Test
  void registerMeasureAcceptsArray() {
    LoggedTunableMeasure<?>[] measures = new LoggedTunableMeasure<?>[0];

    assertDoesNotThrow(() -> TunableUpdaterRegistry.registerMeasure(measures, () -> {}));
  }

  @Test
  void registerMeasureDuplicateIgnored() {
    LoggedTunableMeasure<?>[] measures = new LoggedTunableMeasure<?>[0];

    AtomicInteger first = new AtomicInteger();
    AtomicInteger second = new AtomicInteger();

    TunableUpdaterRegistry.registerMeasure(measures, first::incrementAndGet);
    TunableUpdaterRegistry.registerMeasure(measures, second::incrementAndGet);

    TunableUpdaterRegistry.periodic();

    assertEquals(1, first.get());
    assertEquals(0, second.get());
  }
}
