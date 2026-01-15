package edu.wpi.team190.gompeilib.core.state.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import java.util.List;
import java.util.Optional;

public class Localization {
  private final List<EstimationRegion> estimationRegions;

  private final SwerveDriveOdometry swerveDriveOdometry;

  public Localization(
      List<FieldZone> estimationZones,
      SwerveDriveKinematics kinematics,
      double bufferLengthSeconds) {
    this.swerveDriveOdometry =
        new SwerveDriveOdometry(
            kinematics,
            Rotation2d.kZero,
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            Pose2d.kZero);

    this.estimationRegions =
        estimationZones.stream()
            .map(zone -> new EstimationRegion(zone.aprilTags(), kinematics))
            .toList();
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d rawHeading, SwerveModulePosition[] modulePositions) {
    swerveDriveOdometry.update(rawHeading, modulePositions);
    estimationRegions.forEach(
        region -> region.addOdometryObservation(timestamp, rawHeading, modulePositions));
  }

  public void addPoseObservations(List<VisionPoseObservation> poseObservations) {
    estimationRegions.forEach(
        zone ->
            poseObservations.stream()
                .filter(
                    observation -> zone.getAprilTags().keySet().containsAll(observation.tagIds()))
                .forEach(zone::addPoseObservation));
  }

  public void addTxTyObservations(List<VisionMultiTxTyObservation> txTyObservations) {
    estimationRegions.forEach(
        zone ->
            txTyObservations.stream()
                .filter(observation -> zone.getAprilTags().containsKey(observation.tagId()))
                .forEach(zone::addTxTyObservation));
  }

  public Optional<Pose2d> getEstimatedPose(FieldZone fieldZone) {
    for (EstimationRegion region : estimationRegions) {
      if (fieldZone.aprilTags().equals(region.getAprilTags())) {
        return Optional.of(region.getEstimatedPose());
      }
    }
    return Optional.empty();
  }

  public Rotation2d getHeading() {
    return swerveDriveOdometry.getPoseMeters().getRotation();
  }

  public void resetPose(Pose2d pose) {
    for (EstimationRegion region : estimationRegions) {
      region.resetPose(pose);
    }
    swerveDriveOdometry.resetPose(pose);
  }
}
