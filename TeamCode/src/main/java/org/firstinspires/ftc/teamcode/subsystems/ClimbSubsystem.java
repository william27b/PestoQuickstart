package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.DOWN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class ClimbSubsystem {
    public final CortexLinkedServo piston1;
    public final CortexLinkedServo piston2;

    public final CortexLinkedServo PTOLeft;

    public final CortexLinkedServo PTORight;
    private PistonState pistonState;
    private PTOState ptoState;
    private boolean autoOverride;
    private double currentVelocity;

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

    public enum PTOState {
        ENGAGED (0.5, 0.0),
        NEUTRAL(0.0, 0.65);

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

    public ClimbSubsystem() {
        this.piston1 = MotorCortex.getServo("piston1");
        this.piston2 = MotorCortex.getServo("piston2");

        this.pistonState = PistonState.NEUTRAL;

        this.PTOLeft = MotorCortex.getServo("PTOLeft");
        this.PTORight = MotorCortex.getServo("PTORight");

        this.ptoState = PTOState.NEUTRAL;

        this.update();
    }

    public void setPistonState(PistonState state) {
        this.pistonState = state;
        this.update();
    }

    public void setPTOState(PTOState state) {
        this.ptoState = state;
        this.update();
    }

    public PistonState getPistonState() {
        return this.pistonState;
    }

    public PTOState getPTOState() {
        return this.ptoState;
    }

    public void update() {
        this.PTOLeft.setPosition(this.ptoState.getLeftPosition());
        this.PTORight.setPosition(this.ptoState.getRightPosition());

        this.piston1.setPosition(this.pistonState.getPosition());
        this.piston2.setPosition(this.pistonState.getPosition());
    }

}
