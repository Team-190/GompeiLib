package edu.wpi.team190.gompeilib.core.utility.tunable;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

  // Not sure this is the way to do these, right now the numbers don't actually change when we tell them to over NT, and hasChanged needs to use a hashcode

//  @Test
//  @Order(5)
//  public void hasChangedLastValueNullCurrentValueEqualsLastValue() {
//    GompeiLib.init(null, true, 0.02);
//    LoggedTunableNumber number = new LoggedTunableNumber("number");
//    number.initDefault(42.0);
//    assertTrue(number.hasChanged(0));
//  }
//
//  @Test
//  @Order(6)
//  public void hasChangedLastValueNullCurrentValueNotEqualsLastValue() {
//    GompeiLib.init(null, true, 0.02);
//    LoggedTunableNumber number = new LoggedTunableNumber("number");
//    number.initDefault(42);
//    assertTrue(number.hasChanged(0));
//  }
//
//  @Test
//  @Order(7)
//  public void hasChangedLastValueNotNullCurrentValueEqualsLastValue() {
//    GompeiLib.init(null, true, 0.02);
//    LoggedTunableNumber number = new LoggedTunableNumber("number");
//    number.initDefault(42.0);
//    number.hasChanged(0);
//    assertFalse(number.hasChanged(0));
//  }
}
