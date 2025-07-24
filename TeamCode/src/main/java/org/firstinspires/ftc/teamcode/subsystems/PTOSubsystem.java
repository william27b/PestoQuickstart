package org.firstinspires.ftc.teamcode.subsystems;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class PTOSubsystem {
    public final CortexLinkedServo PTOLeft;
    public final CortexLinkedServo PTORight;

    private PTOState ptoState;

    public enum PTOState {
        ENGAGED (0.5, 0.0),
        NEUTRAL(0.0, 0.65),
        DISENGAGED(0.0, 0.65);

        PTOState(double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }

        private final double leftPos, rightPos;

        public double getLeftPosition() {
            return this.leftPos;
        }
        public double getRightPosition() {
            return this.rightPos;
        }

    }

    public PTOSubsystem() {
        this.PTOLeft = MotorCortex.getServo("PTOLeft");
        this.PTORight = MotorCortex.getServo("PTORight");

        this.ptoState = PTOState.DISENGAGED;

        this.update();
    }

    public void setState(PTOState state) {
        this.ptoState = state;
        this.update();
    }

    public PTOState getState() {
        return this.ptoState;
    }

    public void update() {
        if (ptoState == PTOState.DISENGAGED) {
            this.deactivate();
        } else {
            this.PTOLeft.setPosition(this.ptoState.getLeftPosition());
            this.PTORight.setPosition(this.ptoState.getRightPosition());
        }
    }

    public void deactivate() {
        MotorCortex.ServoCommands.disableServo(this.PTOLeft);
        MotorCortex.ServoCommands.disableServo(this.PTORight);
    }
}
