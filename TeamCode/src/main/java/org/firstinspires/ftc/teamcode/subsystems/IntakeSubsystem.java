package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.STORED;

import static java.lang.Math.pow;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {
    private final CortexLinkedMotor intake;
    private final CortexLinkedServo dropdown;
    private final CortexLinkedServo gate;
    private final ColorSensor colorSensor;

    private IntakeState state;

    public enum IntakeState {
        INTAKE (-1.00, 0.40, 0.49),
        NEUTRAL (0.00, 0.00, 0.49),
        STORING (-0.30, 0.05, 0.49),
        STORING_EMPTY (0.00, 0.05, 0.49),
        STORED (0.00, 0.05, 0.11),
        TO_OUTTAKE (0.00, 0.40, 0.11),
        OUTTAKE (1.00, 0.40, 0.11),
        MEGA_OUTTAKE (-1.00, 0.00, 0.11);

        IntakeState(double power, double dropdownPosition, double gatePosition) {
            this.power = power;
            this.dropdownPosition = dropdownPosition;
            this.gatePosition = gatePosition;
        }

        private final double power;
        private final double dropdownPosition;
        private final double gatePosition;

        public double getPower() {
            return this.power;
        }

        public double getDropdownPosition() {
            return this.dropdownPosition;
        }

        public double getGatePosition() {
            return this.gatePosition;
        }
    }

    public IntakeSubsystem() {
        this.intake = MotorCortex.getMotor("intake");
        this.dropdown = MotorCortex.getServo("dropdown");
        this.gate = MotorCortex.getServo("gate");
        this.colorSensor = (ColorSensor) MotorCortex.hardwareMap.get("colour");

        this.state = STORED;

        this.update();
    }

    public void setState(IntakeState state) {
        this.state = state;
        this.update();
    }

    public IntakeState getState() {
        return this.state;
    }

    public String getSample() {
        double r = colorSensor.red();
        double g = colorSensor.green();
        double b = colorSensor.blue();
        double a = colorSensor.alpha();

        double blue_dist = pow(r - 100, 2) + pow(g - 185, 2) + pow(b - 475, 2) + pow(a - 257, 2);
        double yellow_dist = pow(r - 655, 2) + pow(g - 922, 2) + pow(b - 220, 2) + pow(a - 580, 2);
        double red_dist = pow(r - 420, 2) + pow(g - 220, 2) + pow(b - 127, 2) + pow(a - 242, 2);
        double nothing_dist = pow(r - 42, 2) + pow(g - 62, 2) + pow(b - 60, 2) + pow(a - 55, 2);

        if (blue_dist < yellow_dist && blue_dist < red_dist && blue_dist < nothing_dist)
            return "blue";

        if (yellow_dist < blue_dist && yellow_dist < red_dist && yellow_dist < nothing_dist)
            return "yellow";

        if (red_dist < yellow_dist && red_dist < blue_dist && red_dist < nothing_dist)
            return "red";

        return "nothing";
    }

    public void getColor(Telemetry telemetry) {
        telemetry.addData("r", colorSensor.red());
        telemetry.addData("g", colorSensor.green());
        telemetry.addData("b", colorSensor.blue());
        telemetry.addData("a", colorSensor.alpha());
    }

    private void update() {
        this.intake.setPowerResult(this.state.getPower());
        this.dropdown.setPositionResult(this.state.getDropdownPosition());

        this.gate.setPositionResult(this.state.getGatePosition());
    }
}
