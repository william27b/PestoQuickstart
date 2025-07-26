package org.firstinspires.ftc.teamcode;

import static com.shprobotics.pestocore.hardware.ControlType.ALL;
import static com.shprobotics.pestocore.hardware.ControlType.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.hardware.ControlType;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "TeleOp")
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        MotorCortex.initialize(hardwareMap);

        MecanumController driveController = new MecanumController(
                MotorCortex.getMotor("frontLeft"),
                MotorCortex.getMotor("frontRight"),
                MotorCortex.getMotor("backLeft"),
                MotorCortex.getMotor("backRight")
        );

        driveController.configureMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE
        });

        driveController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TeleOpController teleOpController = new TeleOpController(driveController, hardwareMap);
        teleOpController.setSpeedController(gamepad -> 1.0);

//        DcMotor extendo = (DcMotor) hardwareMap.get("extendo");
//        extendo.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();

//            Vector2D frontLeftPower = new Vector2D(1, 1);
//            Vector2D frontRightPower = new Vector2D(-1, 1);
//            Vector2D backLeftPower = new Vector2D(-1, 1);
//            Vector2D backRightPower = new Vector2D(1, 1);

            if (gamepad1.left_trigger > 0.5) {
                driveController.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                driveController.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                driveController.frontLeft.motor.setPower(0.0);
                driveController.frontRight.motor.setPower(0.0);

                driveController.frontLeft.motor.setControl(AUTO);
                driveController.frontRight.motor.setControl(AUTO);

//                backLeftPower.scale(0.5);
//                backRightPower.scale(0.5);
            } else {
                driveController.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                driveController.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                driveController.frontLeft.motor.setControl(ALL);
                driveController.frontRight.motor.setControl(ALL);
            }

            if (gamepad1.right_trigger > 0.5) {
                driveController.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                driveController.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                driveController.backLeft.motor.setPower(0.0);
                driveController.backRight.motor.setPower(0.0);

                driveController.backLeft.motor.setControl(AUTO);
                driveController.backRight.motor.setControl(AUTO);

//                frontLeftPower.scale(0.5);
//                frontRightPower.scale(0.5);
            } else {
                driveController.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                driveController.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                driveController.backLeft.motor.setControl(ALL);
                driveController.backRight.motor.setControl(ALL);
            }

//            driveController.setPowerVectors(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            extendo.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }
}
