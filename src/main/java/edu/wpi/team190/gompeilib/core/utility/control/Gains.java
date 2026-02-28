package edu.wpi.team190.gompeilib.core.utility.control;

import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public record Gains(
    LoggedTunableNumber kP,
    LoggedTunableNumber kI,
    LoggedTunableNumber kD,
    LoggedTunableNumber kS,
    LoggedTunableNumber kV,
    LoggedTunableNumber kA,
    LoggedTunableNumber kG) {

  @Builder(
      setterPrefix = "with",
      builderMethodName = "fromDoubles",
      builderClassName = "FromDoubles")
  public Gains(
      @NonNull String prefix,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG) {
    this(
        new LoggedTunableNumber(prefix + "/Kp", kP),
        new LoggedTunableNumber(prefix + "/Ki", kI),
        new LoggedTunableNumber(prefix + "/Kd", kD),
        new LoggedTunableNumber(prefix + "/Ks", kS),
        new LoggedTunableNumber(prefix + "/Kv", kV),
        new LoggedTunableNumber(prefix + "/Ka", kA),
        new LoggedTunableNumber(prefix + "/Kg", kG));
  }
}
