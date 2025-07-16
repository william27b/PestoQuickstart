package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem.LinkageState.INTAKE;

import com.shprobotics.pestocore.processing.MotorCortex;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class LinkageSubsystem {
    private final CachingServo linkage;
    private LinkageState state;

    public enum LinkageState {
        OVEREXTENDED (0.5), //from 0.58 to 0.5 for issue #14
        RETRACTED (1.00),
        INTAKE (0.98),
        TRANSFER(0.92),
        SPEC (0.65),
        WALL (0.88); //from 0.9 to 0.88 for issue #14

        LinkageState(double position) {
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return this.position;
        }
    }

    public LinkageSubsystem() {
        this.linkage = MotorCortex.getServo("linkage"); // CortexLinkedServo doesn't work
        this.state = INTAKE;

        this.update();
    }

    public void setState(LinkageState state) {
        this.state = state;
        this.update();
    }

    public LinkageState getState() {
        return this.state;
    }

    private void update() {
        this.linkage.setPosition(this.state.getPosition());
    }
}
