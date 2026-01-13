package edu.wpi.team190.gompeilib.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public ArmIO io;
    public ArmConstants armConstants;
    public ArmIOInputsAutoLogged inputs;

    private boolean isClosedLoop;

    private double position;
    private double positionGoalMeters;

    public Arm(ArmConstants constants, ArmIO io) {
        this.io = io;
        this.armConstants = constants;
        this.inputs = new ArmIOInputsAutoLogged();
    }

    


}
