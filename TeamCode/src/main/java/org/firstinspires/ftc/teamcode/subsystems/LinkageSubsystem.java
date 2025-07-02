package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem.LinkageState.INTAKE;

import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class LinkageSubsystem {
    private final Servo linkage;
    private LinkageState state;

    public enum LinkageState {
        OVEREXTENDED (0.58),
        RETRACTED (1.00),
        INTAKE (0.98),
        WALL (0.9);

        LinkageState(double position) {
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return this.position;
        }
    }

    public LinkageSubsystem() {
        this.linkage = (Servo) MotorCortex.hardwareMap.get("linkage"); // CortexLinkedServo doesn't work
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
