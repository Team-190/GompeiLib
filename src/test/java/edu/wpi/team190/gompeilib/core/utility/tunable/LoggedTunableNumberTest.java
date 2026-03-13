package edu.wpi.team190.gompeilib.core.utility.tunable;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.spy;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import org.junit.jupiter.api.*;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class LoggedTunableNumberTest {
  @BeforeEach
  void setUp() {
    GompeiLib.deinit();
  }

  @Test
  @Order(1)
  public void noDefaultValueTuningMode() {
    GompeiLib.init(null, true, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number");
    number.initDefault(42.0);
    assertEquals(42.0, number.getAsDouble());
  }

  @Test
  @Order(2)
  public void noDefaultValueNoTuningMode() {
    GompeiLib.init(null, false, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number");
    number.initDefault(42.0);
    assertEquals(42.0, number.getAsDouble());
  }

  @Test
  @Order(3)
  public void defaultValueTuningMode() {
    GompeiLib.init(null, true, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number", 190.0);
    number.initDefault(42.0);
    assertEquals(190.0, number.getAsDouble());
  }

  @Test
  @Order(4)
  public void defaultValueNoTuningMode() {
    GompeiLib.init(null, false, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number", 190.0);
    number.initDefault(42.0);
    assertEquals(190.0, number.getAsDouble());
  }

  @Test
  @Order(5)
  public void hasChangedLastValueNullCurrentValueEqualsLastValue() {
    GompeiLib.init(null, true, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number");
    number.initDefault(42.0);
    assertTrue(number.hasChanged(0));
  }

  @Test
  @Order(6)
  public void hasChangedLastValueNullCurrentValueNotEqualsLastValue() {
    GompeiLib.init(null, true, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number");
    number.initDefault(42);
    assertTrue(number.hasChanged(0));
  }

  @Test
  @Order(7)
  public void hasChangedLastValueNotNullCurrentValueEqualsLastValue() {
    GompeiLib.init(null, true, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number");
    number.initDefault(42.0);
    number.hasChanged(0);
    assertFalse(number.hasChanged(0));
  }

  @Test
  @Order(8)
  void testHasChangedWithHashCodes() {
    GompeiLib.init(null, true, 0.02);

    // 1. Setup the tunable and a spy to control the value
    LoggedTunableNumber tunable = new LoggedTunableNumber("Arm/kP", 1.0);
    LoggedTunableNumber spyTunable = spy(tunable);

    // Simulate two different "Subsystems" using their hashcodes as IDs
    int armID = 1001;
    int elevatorID = 2002;

    // 2. Initial state: Both should return true because it's their first time checking
    doReturn(1.0).when(spyTunable).get();
    assertTrue(spyTunable.hasChanged(armID), "Arm should see initial change");
    assertTrue(spyTunable.hasChanged(elevatorID), "Elevator should see initial change");

    // 3. No change: Both should now return false
    assertFalse(spyTunable.hasChanged(armID), "Arm shouldn't see change if value is same");
    assertFalse(
        spyTunable.hasChanged(elevatorID), "Elevator shouldn't see change if value is same");

    // 4. Value Change: Change the dashboard value to 2.0
    doReturn(2.0).when(spyTunable).get();

    // Arm checks first
    assertTrue(spyTunable.hasChanged(armID), "Arm should detect the change to 2.0");
    assertFalse(spyTunable.hasChanged(armID), "Arm second check should be false");

    // Elevator checks second - it should STILL see true because it hasn't acknowledged 2.0 yet
    assertTrue(spyTunable.hasChanged(elevatorID), "Elevator should still see true until it checks");
    assertFalse(spyTunable.hasChanged(elevatorID), "Elevator second check should now be false");
  }

  @Test
  @Order(9)
  void testIfChangedWithMultipleTunables() {
    GompeiLib.init(null, true, 0.02);

    LoggedTunableNumber p = spy(new LoggedTunableNumber("kP", 1.0));
    LoggedTunableNumber i = spy(new LoggedTunableNumber("kI", 0.0));
    int pidControllerID = 555;

    // Initialize tracking
    p.hasChanged(pidControllerID);
    i.hasChanged(pidControllerID);

    final boolean[] updated = {false};

    // Scenario: kP changes, kI stays the same
    doReturn(1.1).when(p).get(); // Change P
    doReturn(0.0).when(i).get(); // Keep I same

    LoggedTunableNumber.ifChanged(pidControllerID, () -> updated[0] = true, p, i);

    assertTrue(updated[0], "The action should run if even one number in the set changes");
  }
}
