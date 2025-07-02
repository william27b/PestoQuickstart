package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.STORED;

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
        INTAKE (-1.00, 0.32, 0.49),
        NEUTRAL (0.00, 0.20, 0.49),
        STORING (0.00, 0.05, 0.49),
        STORED (0.00, 0.05, 0.11),
        OUTTAKE (1.00, 0.32, 0.11);

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
