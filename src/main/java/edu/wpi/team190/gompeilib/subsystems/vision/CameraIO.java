package edu.wpi.team190.gompeilib.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/**
 * An interface for the hardware implementation of a {@link edu.wpi.team190.gompeilib.subsystems.vision.Camera Camera}.
 * Contains the methods necessary for providing information to a camera, like its pipeline,
 * and getting information out of a camera, like its individual robot pose estimate.
 */
public interface CameraIO {
    /**
     * Contains the camera's processed frames,
     * as well as data about its connection and latest update timestamp.
     */
    @AutoLog
    public static class CameraIOInputs {}
}
