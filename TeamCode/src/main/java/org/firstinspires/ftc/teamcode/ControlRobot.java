package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.PestoDashCore;
import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.NumericalRange;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.PestoTelemetry;

import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

@TeleOp(name = "Control Robot", group = "Examples")
public class ControlRobot extends LinearOpMode {
    public void manageServo(NumericalRange range, Servo servo) {
        DataItem item = PestoDashCore.getItem(range.getId());

        if (item == null || item.getValue() == null)
            return;

        double position = (double) item.getValue();

        servo.setPosition(position);
    }

    public void manageCRServo(NumericalRange range, CRServo crServo) {
        DataItem item = PestoDashCore.getItem(range.getId());

        if (item == null || item.getValue() == null)
            return;

        double power = (double) item.getValue();

        crServo.setPower(power);
    }

    public void manageSlides(NumericalRange range, SlideSubsystem slides) {
        DataItem item = PestoDashCore.getItem(range.getId());

        if (item == null || item.getValue() == null)
            return;

        double position = (double) item.getValue();

        slides.setPosition((int) position);
    }

    public void manageExtendo(NumericalRange range, ExtendoSubsystem extendoSubsystem) {
        DataItem item = PestoDashCore.getItem(range.getId());

        if (item == null || item.getValue() == null)
            return;

        double position = (double) item.getValue();

        extendoSubsystem.setPosition((int) position);
    }

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);
        PestoTelemetry pestoTelemetry = FrontalLobe.pestoTelemetry;

        ServoImplEx armLeft = (ServoImplEx) hardwareMap.get("armLeft");
        ServoImplEx armRight = (ServoImplEx) hardwareMap.get("armRight");

        armLeft.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axon PWM
        armRight.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axon PWM

        Servo linkage = (Servo) hardwareMap.get("linkage");

        Servo gate = (Servo) hardwareMap.get("gate");

        Servo claw = (Servo) hardwareMap.get("claw");

        Servo PTOLeft = (Servo) hardwareMap.get("PTOLeft");
        Servo PTORight = (Servo) hardwareMap.get("PTORight");

        Servo piston1 = (Servo) hardwareMap.get("piston1");
        Servo piston2 = (Servo) hardwareMap.get("piston2");

        SlideSubsystem slides = new SlideSubsystem();
        slides.enable();

        ExtendoSubsystem extendoSubsystem = new ExtendoSubsystem();
        extendoSubsystem.enable();

        // sorry the 50.0 is a bug since I forgot to scale the default value for the range
        NumericalRange armPosition = new NumericalRange("Arm Position", 100.0 * 0.04, 0.0, 1.0);
        pestoTelemetry.addToDash(armPosition);

        NumericalRange linkagePosition = new NumericalRange("Linkage Position", 100.0 * 1.0, 0.0, 1.0);
        pestoTelemetry.addToDash(linkagePosition);

        NumericalRange gatePosition = new NumericalRange("Gate Position", 100.0 * 0.11, 0.0, 1.0);
        pestoTelemetry.addToDash(gatePosition);

        NumericalRange clawPosition = new NumericalRange("Claw Position", 100.0 * 1.0, 0.0, 1.0);
        pestoTelemetry.addToDash(clawPosition);

        NumericalRange slidePosition = new NumericalRange("Slide Position", 100.0 * 1.0, -1350, 0.0);
        pestoTelemetry.addToDash(slidePosition);

        NumericalRange extendoPosition = new NumericalRange("Extendo Position", 100.0 * 0.0, 0.0, 640);
        pestoTelemetry.addToDash(extendoPosition);

//        NumericalRange PTOPosition = new NumericalRange("PTO Position", 50.0, 0.0, 1.0);
//        pestoTelemetry.addToDash(PTOPosition);
//
        NumericalRange pistonPosition = new NumericalRange("Piston Position", 50.0, 0.0, 1.0);
        pestoTelemetry.addToDash(pistonPosition);

        pestoTelemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            pestoTelemetry.update();

            manageServo(armPosition, armLeft);
            manageServo(armPosition, armRight);

            manageServo(linkagePosition, linkage); // 0.58 is overextended 1.0 is chill 0.8-0.88 is grab from intake 0.9 is grab from wall

            manageServo(gatePosition, gate);

            manageServo(clawPosition, claw);

            manageSlides(slidePosition, slides);
            manageExtendo(extendoPosition, extendoSubsystem);

//            manageServo(PTOPosition, PTOLeft);
//            manageServo(PTOPosition, PTORight);

//            pistons are cooked #whoforgotthebearings
            manageServo(pistonPosition, piston1);
            manageServo(pistonPosition, piston2);
        }
    }
}
