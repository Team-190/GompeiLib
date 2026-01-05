package edu.wpi.team190.gompeilib.core.state.localization;

import edu.wpi.first.apriltag.AprilTag;

import java.util.Set;

public record FieldZone(Set<AprilTag> aprilTags) {
}
