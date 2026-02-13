package edu.wpi.team190.gompeilib.core.io.components.inertial;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import java.util.Queue;
import lombok.Getter;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  @Getter private final StatusSignal<Angle> yaw;
  @Getter private final StatusSignal<Angle> roll;
  @Getter private final StatusSignal<Angle> pitch;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<AngularVelocity> rollVelocity;
  private final StatusSignal<AngularVelocity> pitchVelocity;

  public GyroIOPigeon2(SwerveDriveConstants driveConstants) {
    Pigeon2 pigeon =
        new Pigeon2(driveConstants.driveConfig.pigeon2Id(), driveConstants.driveConfig.canBus());
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    yaw.setUpdateFrequency(driveConstants.odometryFrequency);
    yawVelocity.setUpdateFrequency(100.0);

    roll = pigeon.getRoll();
    rollVelocity = pigeon.getAngularVelocityZWorld();

    roll.setUpdateFrequency(driveConstants.odometryFrequency);
    rollVelocity.setUpdateFrequency(100.0);

    pitch = pigeon.getRoll();
    pitchVelocity = pigeon.getAngularVelocityZWorld();

    pitch.setUpdateFrequency(driveConstants.odometryFrequency);
    pitchVelocity.setUpdateFrequency(100.0);

    pigeon.optimizeBusUtilization();

    PhoenixUtil.registerSignals(true, yaw, yawVelocity, roll, rollVelocity, pitch, pitchVelocity);
  }

  @Trace
  @Override
  public void updateInputs(
      GyroIOInputs inputs, Queue<Double> yawTimestampQueue, Queue<Double> yawPositionQueue) {
    inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity, roll, rollVelocity, pitch, pitchVelocity);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(rollVelocity.getValueAsDouble());

    inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pitchVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
  }
}
