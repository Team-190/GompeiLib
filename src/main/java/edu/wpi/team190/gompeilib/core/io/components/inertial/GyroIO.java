package edu.wpi.team190.gompeilib.core.io.components.inertial;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import java.util.Queue;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double yawVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void updateInputs(
      GyroIOInputs inputs, Queue<Double> yawTimestampQueue, Queue<Double> yawPositionQueue) {}

  public default StatusSignal<Angle> getYaw() {
    return null;
  }
  ;
}
