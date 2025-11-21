package edu.wpi.team190.gompeilib.subsystems.elevator;
import lombok.getter;

public class Elevator extends SubsystemBase {
    public final ElevatorIO io;
    public final ElevatorIOInputAutoLogged inputs;

    @Getter private ElevatorPositions position;
    private boolean isClosedLoop;

    public Elevator(ElevatorConstants elevatorConstants, ElevatorIO io) {
        this.elevatorConstants = elevatorConstants;
        this.io = io;
    }

    @Trace
    public void periodic() {
        elevatorConstants.lock.lock();
        io.updateInputs(inputs);
        InternalLoggedTracer.record("Update Inputs", "Elevator/Periodic")
        elevatorConstants.lock.unlock();

        Logger.processInputs("Elevator", inputs);

        Logger.recordOutput("Elevator/Position", position.name());
        if (isClosedLoop) {
            io.setPositionGoal(position.getPosition);
        }
        Logger.record("Set Position Goal", "Elevator/Periodic");
    }

    private void setGains(double kP, double kD, double kS, double kV, doubla kA, double kG) {
        io.updateGains(kP, kD, kS, kV, kA, kG);
    }


}
