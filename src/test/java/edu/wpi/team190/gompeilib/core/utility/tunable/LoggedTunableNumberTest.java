package edu.wpi.team190.gompeilib.core.utility.tunable;

import static org.junit.jupiter.api.Assertions.assertEquals;

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

  }

  @Test
  @Order(6)
  public void hasChangedLastValueNullCurrentValueNotEqualsLastValue() {
    GompeiLib.init(null, true, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number");

  }

  @Test
  @Order(7)
  public void hasChangedLastValueNotNullCurrentValueEqualsLastValue() {
    GompeiLib.init(null, true, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number");

  }

  @Test
  @Order(8)
  public void hasChangedLastValueNotNullCurrentValueNotEqualsLastValue() {
    GompeiLib.init(null, true, 0.02);
    LoggedTunableNumber number = new LoggedTunableNumber("number");

  }
}
