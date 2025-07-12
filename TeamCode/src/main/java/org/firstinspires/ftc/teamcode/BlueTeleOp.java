package org.firstinspires.ftc.teamcode;

import static com.shprobotics.pestocore.devices.GamepadKey.A;
import static com.shprobotics.pestocore.devices.GamepadKey.B;
import static com.shprobotics.pestocore.devices.GamepadKey.DPAD_DOWN;
import static com.shprobotics.pestocore.devices.GamepadKey.DPAD_RIGHT;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_TRIGGER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_TRIGGER;
import static com.shprobotics.pestocore.devices.GamepadKey.TOUCHPAD;
import static com.shprobotics.pestocore.devices.GamepadKey.Y;
import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.SpecState.HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.SpecState.TO_WALL;
import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem.ExtendoState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem.ExtendoState.OUT;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

@TeleOp(name = "Blue TeleOp")
public class BlueTeleOp extends BaseRobot {
    @Override
    public void runOpMode() {
        boolean justBlue = true;

        super.runOpMode();
        waitForStart();

        extendoSubsystem.enable();
        slideSubsystem.enable();

        while (opModeIsActive() && !isStopRequested()) {
            gamepadInterface1.update();
            MotorCortex.update();
            FrontalLobe.update();

            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            Vector2D currentPosition = tracker.getCurrentPosition().asVector();
            telemetry.addData("X", currentPosition.getX());
            telemetry.addData("Y", currentPosition.getY());
            telemetry.addData("Rotation", tracker.getCurrentPosition().getHeadingRadians());

            if (gamepad1.b) {
                tracker.reset();
            }

            if(gamepad1.dpad_left){
                teleOpController.resetIMU();
            }

            if (gamepadInterface1.isKeyDown(Y) || (specState == TO_WALL && !clawSubsystem.breakbeam.getState())) {
                switch (specState) {
                    case RELEASE:
                        specState = SpecState.TO_WALL;
                        break;
                    case TO_WALL:
                        specState = HIGH_RUNG;
                        break;
                    case HIGH_RUNG:
                        specState = SpecState.RELEASE;
                        break;
                }

                FrontalLobe.removeMacros("spec");
                FrontalLobe.useMacro(specState.getMacroAlias());
            }

            if (gamepadInterface1.isKey(DPAD_DOWN)) {
                slideSubsystem.topSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideSubsystem.botSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                while (gamepadInterface1.isKey(DPAD_DOWN) && opModeIsActive() && !isStopRequested()) {
                    slideSubsystem.topSlide.setPowerResult(0.6);
                    slideSubsystem.botSlide.setPowerResult(0.6);

                    gamepadInterface1.update();
                }

                slideSubsystem.topSlide.setPowerResult(0.0);
                slideSubsystem.botSlide.setPowerResult(0.0);

                slideSubsystem.topSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideSubsystem.botSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                slideSubsystem.setState(SlideSubsystem.SlideState.DOWN);

                slideSubsystem.topSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideSubsystem.botSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slideSubsystem.enable();
            }

            if (gamepadInterface1.isKey(DPAD_RIGHT)) {
                extendoSubsystem.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                while (gamepadInterface1.isKey(DPAD_RIGHT) && opModeIsActive() && !isStopRequested()) {
                    extendoSubsystem.extendo.setPowerResult(-0.6);
                    gamepadInterface1.update();
                }

                extendoSubsystem.extendo.setPowerResult(0.0);
                extendoSubsystem.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendoSubsystem.setState(IN);
                extendoSubsystem.extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendoSubsystem.enable();
            }

            // checks for the moment the key is pressed
            if (gamepadInterface1.isKeyDown(LEFT_TRIGGER)) {
                if (slideSubsystem.getState() == SlideSubsystem.SlideState.DOWN)
                    clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                if (extendoSubsystem.getState() == IN) {
                    extendoSubsystem.setState(OUT); // toggle extension
                } else {
                    FrontalLobe.useMacro("intake - store");
                }
            }

            boolean shouldRejectSample = intakeSubsystem.getSample().equals("red") || (intakeSubsystem.getSample().equals("yellow") && justBlue);
            if (extendoSubsystem.getState() == OUT) {
                if (shouldRejectSample)
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.MEGA_OUTTAKE);
                else if (gamepadInterface1.isKey(A))
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTTAKE);
                else if (gamepadInterface1.isKey(RIGHT_TRIGGER))
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                else if (extendoSubsystem.getPosition() < 540)
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRALIZING);
                else
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
            }

            if ((intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORED || intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORING) && gamepadInterface1.isKeyDown(RIGHT_BUMPER)) {
                switch (transferState) {
                    case RELEASING:
                    case FROM_SUB:
                        transferState = TransferState.TRANSFERRING;
                        break;
                    case TRANSFERRING:
                        transferState = TransferState.RELEASING;
                        break;
                }

                FrontalLobe.useMacro(transferState.getMacroAlias());
            }

            if ((intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORED || intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORING) && gamepadInterface1.isKeyDown(B)) {
                switch (transferState) {
                    case RELEASING:
                    case FROM_SUB:
                        transferState = TransferState.TO_SUB;
                        break;
                    case TO_SUB:
                        transferState = TransferState.FROM_SUB;
                        break;
                }

                FrontalLobe.useMacro(transferState.getMacroAlias());
            }

            if (gamepadInterface1.isKeyDown(TOUCHPAD))
                justBlue = !justBlue;

            if (justBlue)
                gamepad1.setLedColor(0, 0, 255, Integer.MAX_VALUE);
            else
                gamepad1.setLedColor(255, 255, 0, Integer.MAX_VALUE);

            telemetry.addData("velocity", slideSubsystem.getVelocity());
            telemetry.addData("position", slideSubsystem.getPosition());
            telemetry.update();
            tracker.update();
        }
    }
}
