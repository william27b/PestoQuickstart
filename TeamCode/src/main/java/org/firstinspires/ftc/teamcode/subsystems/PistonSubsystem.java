package org.firstinspires.ftc.teamcode.subsystems;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class PistonSubsystem {
    public final CortexLinkedServo piston1;
    public final CortexLinkedServo piston2;

    private PistonState pistonState;

    public enum PistonState {
        UP (0.0),
        NEUTRAL(0.6),
        DOWN(1.0);

        PistonState(double position) {
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return this.position;
        }
    }

    public PistonSubsystem() {
        this.piston1 = MotorCortex.getServo("piston1");
        this.piston2 = MotorCortex.getServo("piston2");

        this.pistonState = PistonState.NEUTRAL;

        this.update();
    }

    public void setState(PistonState state) {
        this.pistonState = state;
        this.update();
    }

    public PistonState getState() {
        return this.pistonState;
    }

    public void update() {
        if (pistonState == PistonState.NEUTRAL) {
            MotorCortex.ServoCommands.disableServo(this.piston1);
            MotorCortex.ServoCommands.disableServo(this.piston2);
        } else {
            this.piston1.setPosition(this.pistonState.getPosition());
            this.piston2.setPosition(this.pistonState.getPosition());
        }
    }

    public void deactivate() {
        MotorCortex.ServoCommands.disableServo(this.piston1);
        MotorCortex.ServoCommands.disableServo(this.piston2);
    }
}
