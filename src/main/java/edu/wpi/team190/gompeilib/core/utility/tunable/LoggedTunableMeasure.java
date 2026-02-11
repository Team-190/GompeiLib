package edu.wpi.team190.gompeilib.core.utility.tunable;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable measure (unit-safe). Gets value from dashboard in tuning mode, returns
 * default if not or value not in dashboard.
 */
public class LoggedTunableMeasure<U extends Unit> implements Supplier<Measure<U>> {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private final U unit;
  private boolean hasDefault = false;
  private Measure<U> defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Measure<U>> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableMeasure with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default measure value
   */
  public LoggedTunableMeasure(String dashboardKey, Measure<U> defaultValue) {
    this.key = tableKey + "/" + dashboardKey;
    this.unit = defaultValue.unit();
    initDefault(defaultValue);
  }

  /** Set the default value. The default value can only be set once. */
  private void initDefault(Measure<U> defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (GompeiLib.isTuning()) {
        // We store the raw double value in the base unit
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue.in(unit));
      }
    }
  }

  /** Get the current measure, from dashboard if available and in tuning mode. */
  @Override
  @SuppressWarnings("unchecked") // safe
  public Measure<U> get() {
    if (!hasDefault) {
      return (Measure<U>) unit.zero();
    } else {
      double rawValue = GompeiLib.isTuning() ? dashboardNumber.get() : defaultValue.in(unit);
      return (Measure<U>) unit.of(rawValue);
    }
  }

  /** Checks whether the measure has changed since our last check */
  public boolean hasChanged(int id) {
    Measure<U> currentValue = get();
    Measure<U> lastValue = lastHasChangedValues.get(id);
    if (!currentValue.equals(lastValue) || lastValue == null) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  /** Runs action if any of the tunableMeasures have changed */
  @SafeVarargs
  public static <U extends Unit> void ifChanged(
      int id, Consumer<Measure<U>[]> action, LoggedTunableMeasure<U>... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(n -> n.hasChanged(id))) {
      @SuppressWarnings("unchecked")
      Measure<U>[] values =
          Arrays.stream(tunableNumbers).map(LoggedTunableMeasure::get).toArray(Measure[]::new);
      action.accept(values);
    }
  }

  public static void ifChanged(int id, Runnable action, LoggedTunableMeasure<?>... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(n -> n.hasChanged(id))) {
      action.run();
    }
  }
}
