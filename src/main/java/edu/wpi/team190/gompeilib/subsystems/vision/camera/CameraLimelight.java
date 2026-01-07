package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.LimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionSingleTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.LimelightIOInputsAutoLogged;
import java.util.*;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CameraLimelight extends Camera {
  private final LimelightIOInputsAutoLogged inputs;
  private final CameraIOLimelight io;

  private final LimelightConfig config;

  @Getter private final String name;

  @Getter private final List<Pose3d> allTagPoses;

  public CameraLimelight(
      CameraIOLimelight io,
      LimelightConfig config,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    super(config.key(), poseObservers, new ArrayList<>(), singleTxTyObservers);

    inputs = new LimelightIOInputsAutoLogged();
    this.io = io;

    this.config = config;
    this.name = config.key();

    allTagPoses = new ArrayList<>();

    currentCameraPose =
        new Pose3d(
            config.robotToCameraTransform().getTranslation(),
            config.robotToCameraTransform().getRotation());
  }

  @Override
  public void periodic() {
    poseObservationList.clear();
    multiTxTyObservationList.clear();
    singleTxTyObservationList.clear();

    io.updateInputs(inputs);
    Logger.processInputs("Vision/Cameras/" + this.name, inputs);

    allTagPoses.clear();

    double xyStdDev =
        config.megatagXYStdev()
            * Math.pow(inputs.mt1PoseEstimate.avgTagDist(), 1.2)
            / Math.pow(inputs.mt1PoseEstimate.tagCount(), 2);
    double thetaStdev =
        inputs.mt1PoseEstimate.tagCount() > 1
            ? config.metatagThetaStdev()
                * Math.pow(inputs.mt1PoseEstimate.avgTagDist(), 1.2)
                / Math.pow(inputs.mt1PoseEstimate.tagCount(), 2)
            : Double.POSITIVE_INFINITY;

    poseObservationList.add(
        new VisionPoseObservation(
            inputs.mt1PoseEstimate.pose(),
            Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                .map(CameraIO.RawFiducial::id)
                .collect(Collectors.toSet()),
            inputs.mt1PoseEstimate.timestampSeconds(),
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));

    xyStdDev =
        config.megatag2XYStdev()
            * Math.pow(inputs.mt1PoseEstimate.avgTagDist(), 1.2)
            / Math.pow(inputs.mt1PoseEstimate.tagCount(), 2);
    thetaStdev = Double.POSITIVE_INFINITY;

    poseObservationList.add(
        new VisionPoseObservation(
            inputs.mt2PoseEstimate.pose(),
            Arrays.stream(inputs.mt2PoseEstimate.rawFiducials())
                .map(CameraIO.RawFiducial::id)
                .collect(Collectors.toSet()),
            inputs.mt2PoseEstimate.timestampSeconds(),
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));

    Arrays.stream(inputs.rawFiducials)
        .forEach(
            fiducial ->
                singleTxTyObservationList.add(
                    new VisionSingleTxTyObservation(
                        fiducial.id(),
                        fiducial.txnc(),
                        fiducial.tync(),
                        fiducial.distToCamera(),
                        inputs.mt1PoseEstimate.timestampSeconds(),
                        currentCameraPose)));

    super.sendObservers();
  }
}
