package edu.wpi.team190.gompeilib.subsystems.vision.io;

import edu.wpi.team190.gompeilib.subsystems.vision.camera.Camera;
import org.littletonrobotics.junction.AutoLog;

/**
 * An interface for the hardware implementation of a {@link Camera Camera}. Contains the methods
 * necessary for providing information to a camera, like its pipeline, and getting information out
 * of a camera, like its individual robot pose estimate.
 */
public interface CameraIO {

  @AutoLog
  public static class GompeiVisionIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public double captureFPS = 0;
    public double processingFPS = 0;
  }

  public static class LimelightIOInputs {}

  public default void updateInputs(GompeiVisionIOInputs inputs) {}

  public default void updateInputs(LimelightIOInputs inputs) {}

  public default String getName() {
    return "";
  }
}
