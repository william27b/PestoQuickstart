package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.PestoDashCore;
import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.NumericalRange;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.PestoTelemetry;

@TeleOp(name = "Control Axons", group = "Examples")
public class ControlAxons extends LinearOpMode {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);
        PestoTelemetry pestoTelemetry = FrontalLobe.pestoTelemetry;

        // sorry the 50.0 is a bug since I forgot to scale the default value for the range
        NumericalRange servoPower = new NumericalRange("Axon Power", 50.0, -1.0, 1.0);
        pestoTelemetry.addToDash(servoPower);
        pestoTelemetry.update();

        ServoImplEx servo1 = (ServoImplEx) hardwareMap.get("servo1"); // Change servo names in quotes to match config names
        ServoImplEx servo2 = (ServoImplEx) hardwareMap.get("servo2"); // Change servo names in quotes to match config names

        // this is the default PWM for Axons
        servo1.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Change pwm if necessary
        servo2.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Change pwm if necessary

        servo1.setDirection(Servo.Direction.FORWARD); // Change to Servo.Direction.REVERSE if necessary
        servo2.setDirection(Servo.Direction.FORWARD); // Change to Servo.Direction.REVERSE if necessary

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            pestoTelemetry.update();


            DataItem item = PestoDashCore.getItem(servoPower.getId());

            if (item == null || item.getValue() == null) {
                telemetry.update();
                continue;
            }

            double power = (double) item.getValue();

            servo1.setPosition(power);
            servo2.setPosition(power);
        }
    }
}
