package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import java.util.Set;
import lombok.Builder;
import lombok.Singular;

@Builder
public class GenericFlywheelConstants {

  public final int LEADER_CAN_ID;
  public final InvertedValue LEADER_INVERSION;

  @Builder.Default public final CANBus CAN_LOOP = new CANBus();
  @Builder.Default public final boolean ENABLE_FOC = false;

  public final double CURRENT_LIMIT;
  public final double MOMENT_OF_INERTIA;
  public final double GEAR_RATIO;

  public final DCMotor MOTOR_CONFIG;

  public final Gains GAINS;
  public final Constraints CONSTRAINTS;

  @Singular(value = "ALIGNED_FOLLOWER_CAN_ID")
  public final Set<Integer> ALIGNED_FOLLOWER_CAN_IDS;

  @Singular(value = "OPPOSED_FOLLOWER_CAN_ID")
  public final Set<Integer> OPPOSED_FOLLOWER_CAN_IDS;

  @Builder
  public record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public record Constraints(
      LoggedTunableNumber maxAccelerationRadiansPerSecondSquared,
      LoggedTunableNumber cruisingVelocityRadiansPerSecond,
      LoggedTunableNumber goalToleranceRadiansPerSecond) {}
}
