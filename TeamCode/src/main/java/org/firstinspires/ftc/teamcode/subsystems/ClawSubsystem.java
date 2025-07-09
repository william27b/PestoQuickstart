package org.firstinspires.ftc.teamcode.subsystems;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class ClawSubsystem {
    private final CortexLinkedServo claw;
    private ClawState state;

    public enum ClawState {
        OPEN (1.00),
        CLOSED (0.635);

        ClawState(double position) {
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return this.position;
        }
    }

    public ClawSubsystem() {
        this.claw = MotorCortex.getServo("claw");
        this.state = ClawState.OPEN;

        this.update();
    }

    public void setState(ClawState state) {
        this.state = state;
        this.update();
    }

    public ClawState getState() {
        return this.state;
    }

    private void update() {
        this.claw.setPosition(this.state.getPosition());
    }
}
