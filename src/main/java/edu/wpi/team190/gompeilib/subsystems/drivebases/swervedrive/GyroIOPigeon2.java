package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

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

import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {

    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOPigeon2(DriveConstants driveConstants) {
      Pigeon2 pigeon = new Pigeon2(driveConstants.DRIVE_CONFIG.pigeon2Id(), driveConstants.DRIVE_CONFIG.canBus());
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    yaw.setUpdateFrequency(driveConstants.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);

    pigeon.optimizeBusUtilization();

    yawTimestampQueue = PhoenixOdometryThread.getInstance(driveConstants).makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance(driveConstants).registerSignal(pigeon.getYaw());

    PhoenixUtil.registerSignals(true, yaw, yawVelocity);
  }

  @Trace
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map(Rotation2d::fromDegrees)
            .toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
