package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.shprobotics.pestocore.processing.MotorCortex;

public class ArmSubsystem {
    private final ServoImplEx armLeft;
    private final ServoImplEx armRight;
    private ArmState state;

    public enum ArmState {
        WALL (0.045),
        BUCKET (0.16),
        CLIMB(0.35),
        DEPOSIT (0.74),
        TRANSFER (0.98);

        ArmState(double position) {
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return this.position;
        }
    }

    public ArmSubsystem() {
        this.armLeft = (ServoImplEx) MotorCortex.hardwareMap.get("armLeft");
        this.armLeft.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axon PWM

        this.armRight = (ServoImplEx) MotorCortex.hardwareMap.get("armLeft");
        this.armRight.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axon PWM

        this.state = ArmState.TRANSFER;

        this.update();
    }

    public void setState(ArmState state) {
        this.state = state;
        this.update();
    }

    public ArmState getState() {
        return this.state;
    }

    public void update() {
        this.armLeft.setPosition(this.state.getPosition());
        this.armRight.setPosition(this.state.getPosition());
    }

    public void deactivate() {
    }
}
