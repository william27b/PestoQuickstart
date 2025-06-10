package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.PestoDashCore;
import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.NumericalRange;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.PestoTelemetry;

@TeleOp(name = "ControlServos", group = "Examples")
public class ControlServos extends LinearOpMode {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);
        PestoTelemetry pestoTelemetry = FrontalLobe.pestoTelemetry;

        // sorry the 50.0 is a bug since I forgot to scale the default value for the range
        NumericalRange servoPower = new NumericalRange("Servo Power", 50.0, -1.0, 1.0);
        pestoTelemetry.addToDash(servoPower);
        pestoTelemetry.update();

        Servo servoTop = (Servo) hardwareMap.get("servoTop");
        Servo servoBottom = (Servo) hardwareMap.get("servoBottom");

//        DcMotor left = (DcMotor) hardwareMap.get("frontLeft");
//        DcMotor right = (DcMotor) hardwareMap.get("frontRight");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            pestoTelemetry.update();


            DataItem item = PestoDashCore.getItem(servoPower.getId());

            if (item == null || item.getValue() == null) {
                telemetry.update();
                continue;
            }

            double power = (double) item.getValue();

            servoTop.setPosition(power);
            servoBottom.setPosition(power);

//            left.setPower(power);
//            right.setPower(power);
        }
    }
}
