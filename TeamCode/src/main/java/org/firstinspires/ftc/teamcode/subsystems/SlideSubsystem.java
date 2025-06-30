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

    public enum SlideState {
        DOWN (0),
        UP (-1350);

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
    }

    public void setState(SlideState state) {
        this.state = state;
        this.update();
    }

    public SlideState getState() {
        return this.state;
    }

    public double getPosition() {
        return this.botSlide.getCurrentPosition();
    }

    private void update() {
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
