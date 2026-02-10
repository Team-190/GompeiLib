package edu.wpi.team190.gompeilib.core.utility;

import lombok.Builder;
import lombok.NonNull;

public record Gains(
    LoggedTunableNumber kP,
    LoggedTunableNumber kI,
    LoggedTunableNumber kD,
    LoggedTunableNumber kS,
    LoggedTunableNumber kV,
    LoggedTunableNumber kA,
    LoggedTunableNumber kG) {

  @Builder(setterPrefix = "with")
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
