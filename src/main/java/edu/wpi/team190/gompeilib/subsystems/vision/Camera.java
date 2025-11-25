package edu.wpi.team190.gompeilib.subsystems.vision;

/**
 * A virtual subsystem class that holds the IO for any camera type.
 */
public class Camera {

    /**
     * A record containing the information about each frame a camera processes.
     * The camera takes in information about the tag and converts it into useful info,
     * like an estimated position on the field or a distance from the tag. This record stores
     * all the information that might be used in the code for localization.
     */
    public record ProcessedFrame() {}
}
