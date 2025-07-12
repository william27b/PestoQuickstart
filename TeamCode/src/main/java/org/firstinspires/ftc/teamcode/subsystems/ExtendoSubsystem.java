package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem.ExtendoState.IN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

public class ExtendoSubsystem {
    public final CortexLinkedMotor extendo;
    private ExtendoState state;

    public enum ExtendoState {
        IN (0),
        MIN_OUT (102),
        OUT (640);

        ExtendoState(int position) {
            this.position = position;
        }

        private final int position;

        public int getPosition() {
            return this.position;
        }
    }

    public ExtendoSubsystem() {
        this.extendo = MotorCortex.getMotor("extendo");
        this.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.state = IN;
        this.update();

        this.extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setState(ExtendoState state) {
        this.state = state;
        this.update();
    }

    public ExtendoState getState() {
        return this.state;
    }

    public void setPosition(int position) {
        this.extendo.setTargetPosition(position);
    }

    public double getPosition() {
        return this.extendo.getCurrentPosition();
    }

    private void update() {
        this.extendo.setTargetPosition(this.state.getPosition());
    }

    public void enable() {
        this.extendo.setPowerResult(0.8);
    }

    public void disable() {
        this.extendo.setPowerResult(0.0);
    }
}
