package org.firstinspires.ftc.teamcode;

import static com.shprobotics.pestocore.devices.GamepadKey.A;
import static com.shprobotics.pestocore.devices.GamepadKey.DPAD_DOWN;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_TRIGGER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_TRIGGER;
import static com.shprobotics.pestocore.devices.GamepadKey.Y;
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
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;

@TeleOp(name = "Drive")
public class Drive extends BaseRobot {
    @Override
    public void runOpMode() {
        super.runOpMode();
        waitForStart();

        extendoSubsystem.enable();
        slideSubsystem.enable();

        while (opModeIsActive() && !isStopRequested()) {
            gamepadInterface1.update();
            MotorCortex.update();
            FrontalLobe.update();

            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            Vector2D currentPosition = tracker.getCurrentPosition().asVector();
            telemetry.addData("X", currentPosition.getX());
            telemetry.addData("Y", currentPosition.getY());
            telemetry.addData("Rotation", tracker.getCurrentPosition().getHeadingRadians());

            if (gamepad1.b) {
                tracker.reset();
            }

            if (gamepadInterface1.isKey(DPAD_DOWN)) {
                extendoSubsystem.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                while (gamepadInterface1.isKey(DPAD_DOWN) && opModeIsActive() && !isStopRequested()) {
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
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                if (extendoSubsystem.getState() == IN) {
                    extendoSubsystem.setState(OUT); // toggle extension
                } else {
                    FrontalLobe.useMacro("intake - store");
                }
            }

            if (extendoSubsystem.getState() == OUT) {
                if (gamepadInterface1.isKey(RIGHT_TRIGGER))
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                else if (gamepadInterface1.isKey(A))
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTTAKE);
                else
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
            }

            if ((intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORED || intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORING) && gamepadInterface1.isKeyDown(RIGHT_BUMPER)) {
                switch (transferState) {
                    case RETURNING:
                        transferState = TransferState.TRANSFERRING;
                        break;
                    case TRANSFERRING:
                        transferState = TransferState.RELEASING;
                        break;
                    case RELEASING:
                        transferState = TransferState.RETURNING;
                        break;
                }

                FrontalLobe.useMacro(transferState.getMacroAlias());
            }

            if (gamepadInterface1.isKeyDown(Y)) {
                if (linkageSubsystem.getState() != LinkageSubsystem.LinkageState.OVEREXTENDED) {
                    clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                    linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);
                } else if (clawSubsystem.getState() != ClawSubsystem.ClawState.CLOSED) {
                    clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
                } else {
                    linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
                }
            }

            intakeSubsystem.getColor(telemetry);
            telemetry.addData("position", slideSubsystem.getPosition());
            telemetry.update();
            tracker.update();
        }
    }
}
