package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class ClawSubsystem {
    private final CortexLinkedServo claw;
    public final DigitalChannel breakbeam;
    private ClawState state;

    public enum ClawState {
        OPEN (0.90),
        CLOSED (0.55);

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
        this.breakbeam = (DigitalChannel) MotorCortex.hardwareMap.get("beam");
        this.state = ClawState.CLOSED;

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
