package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.DOWN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

public class SlideSubsystem {
    public final CortexLinkedMotor botSlide;
    public final CortexLinkedMotor topSlide;
    private SlideState state;

    private double currentVelocity;

    public enum SlideState {
        DOWN (0),
        MEDIUM(-400),
        SPEC (-865),
        UP (-1250);

        SlideState(int position) {
            this.position = position;
        }

        private final int position;

        public int getPosition() {
            return this.position;
        }
    }

    public SlideSubsystem() {
        this.botSlide = MotorCortex.getMotor("botSlide");
        this.botSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.botSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        this.topSlide = MotorCortex.getMotor("topSlide");
        this.topSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.topSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        this.state = DOWN;
        this.update();

        this.botSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.topSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.currentVelocity = 0.0;
    }

    public void setState(SlideState state) {
        this.state = state;
        this.update();
    }

    public SlideState getState() {
        return this.state;
    }

    public void setPosition(int position) {
        this.botSlide.setTargetPosition(position);
        this.topSlide.setTargetPosition(position);
    }

    public double getPosition() {
        return this.botSlide.getCurrentPosition();
    }

    public double getVelocity() {
        return this.botSlide.getVelocity();
    }

    private void update() {
        currentVelocity = (currentVelocity * 0.9) + (this.getVelocity());

        if (this.state == DOWN && currentVelocity > 50) {
            this.botSlide.setPowerResult(0.0);
            this.topSlide.setPowerResult(0.0);
        } else {
            this.botSlide.setPowerResult(0.6);
            this.topSlide.setPowerResult(0.6);
        }

        this.botSlide.setTargetPosition(this.state.getPosition());
        this.topSlide.setTargetPosition(this.state.getPosition());
    }

    public void enable() {
        this.botSlide.setPowerResult(0.6);
        this.topSlide.setPowerResult(0.6);
    }

    public void disable() {
        this.botSlide.setPowerResult(0.0);
        this.topSlide.setPowerResult(0.0);
    }
}
