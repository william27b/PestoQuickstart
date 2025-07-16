package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ArmSubsystem {
    private final ServoImplEx armLeft;
    private final ServoImplEx armRight;
    private ArmState state;

    public enum ArmState {
        WALL (0.02), //from 0.04 to 0.02 for issue #14
        BUCKET (0.16),
        DEPOSIT (0.76),
        TRANSFER (0.96);

        ArmState(double position) {
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return this.position;
        }
    }

    public ArmSubsystem(HardwareMap hardwareMap) {
        this.armLeft = (ServoImplEx) hardwareMap.get("armLeft");
        this.armLeft.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axon PWM

        this.armRight = (ServoImplEx) hardwareMap.get("armLeft");
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

    private void update() {
        this.armLeft.setPosition(this.state.getPosition());
        this.armRight.setPosition(this.state.getPosition());
    }
}
