package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionTxTyObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import lombok.Getter;

public abstract class Camera {
  @Getter private final String name;
  List<Consumer<List<VisionPoseObservation>>> poseObservers;
  List<Consumer<List<VisionTxTyObservation>>> txTyObservers;

  @Getter List<VisionPoseObservation> poseObservationList;
  @Getter List<VisionTxTyObservation> txTyObservationList;

  public Camera(
      String name,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionTxTyObservation>>> txTyObservers) {
    this.name = name;
    this.poseObservers = poseObservers;
    this.txTyObservers = txTyObservers;

    poseObservationList = new ArrayList<>();
    txTyObservationList = new ArrayList<>();
  }

  @Trace
  public void periodic() {}

  public void sendObservers() {
    for (Consumer<List<VisionPoseObservation>> observer : poseObservers) {
      observer.accept(poseObservationList);
    }
    for (Consumer<List<VisionTxTyObservation>> observer : txTyObservers) {
      observer.accept(txTyObservationList);
    }
  }
}
