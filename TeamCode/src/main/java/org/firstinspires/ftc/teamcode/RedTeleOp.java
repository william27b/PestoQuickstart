package org.firstinspires.ftc.teamcode;

import static com.shprobotics.pestocore.devices.GamepadKey.A;
import static com.shprobotics.pestocore.devices.GamepadKey.B;
import static com.shprobotics.pestocore.devices.GamepadKey.DPAD_DOWN;
import static com.shprobotics.pestocore.devices.GamepadKey.DPAD_RIGHT;
import static com.shprobotics.pestocore.devices.GamepadKey.DPAD_UP;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_TRIGGER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_TRIGGER;
import static com.shprobotics.pestocore.devices.GamepadKey.TOUCHPAD;
import static com.shprobotics.pestocore.devices.GamepadKey.Y;
import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.TransferState.CLIMB_SLIDES_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.TransferState.CLIMB_SLIDES_UP;
import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.TransferState.HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.TransferState.PISTON;
import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.TransferState.RELEASE_FROM_SPEC;
import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.TransferState.SPEC_WALL;
import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.TransferState.SPEC_WALL_FROM_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem.ExtendoState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem.ExtendoState.OUT;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.CLIMB;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.CLIMB_UP;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.UP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

@TeleOp(name = "Red TeleOp")
public class RedTeleOp extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        boolean justRed = true;

        super.runOpMode();
        waitForStart();

        extendoSubsystem.enable();
        slideSubsystem.enable();

        while (opModeIsActive() && !isStopRequested()) {
            gamepadInterface1.update();
            MotorCortex.update();
            FrontalLobe.update();

            teleOpController.driveFieldCentric(0.9*-gamepad1.left_stick_y, 0.9*gamepad1.left_stick_x, 0.9*gamepad1.right_stick_x);

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

            // SPEC HIGH RUNG STATES
            if (gamepadInterface1.isKeyDown(Y) || ((transferState == SPEC_WALL || transferState == SPEC_WALL_FROM_RUNG) && !clawSubsystem.breakbeam.getState())) {
                switch (transferState) {
                    case RELEASE_FROM_SPEC:
                    case RELEASE_FROM_SAMPLE:
                    case BUCKET_TRANSFERRING:
                        transferState = SPEC_WALL;
                        break;
                    case HIGH_RUNG:
                        transferState = SPEC_WALL_FROM_RUNG;
                        break;
                    case SPEC_WALL_FROM_RUNG:
                    case SPEC_WALL:
                        transferState = HIGH_RUNG;
                        break;
                }

                FrontalLobe.removeMacros("spec");
                FrontalLobe.useMacro(transferState.getMacroAlias());
            }

            // reset the vertical slides position
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

            // reset the extendo position
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
            // toggles the extendo out or in
            if (gamepadInterface1.isKeyDown(LEFT_TRIGGER)) {
                if (slideSubsystem.getState() == SlideSubsystem.SlideState.DOWN)
                    clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                if (extendoSubsystem.getState() == IN) {
                    extendoSubsystem.setState(OUT); // toggle extension
                } else {
                    FrontalLobe.useMacro("intake - store");
                }
            }

            // COLOR SENSOR LOGIC
            boolean shouldRejectSample = intakeSubsystem.getSample().equals("blue") || (intakeSubsystem.getSample().equals("yellow") && justRed);
            boolean shouldAccept = !shouldRejectSample && !intakeSubsystem.getSample().equals("nothing");
            if (extendoSubsystem.getState() == OUT) {
                if (shouldRejectSample) {
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.MEGA_OUTTAKE);
                    FrontalLobe.removeMacros("intake");
                } else if (gamepadInterface1.isKey(A)) {
                    if (!FrontalLobe.hasMacro("intake - outtake"))
                        FrontalLobe.useMacro("intake - outtake");
                } else if (shouldAccept) {
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORING);
                    FrontalLobe.removeMacros("intake");
                } else if (gamepadInterface1.isKey(RIGHT_TRIGGER)) {
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                    FrontalLobe.removeMacros("intake");
                } else {
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                    FrontalLobe.removeMacros("intake");
                }
            }

            if (!shouldAccept && intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORING)
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORING_EMPTY);
            else if (shouldAccept && intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORING_EMPTY)
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORING);

            if ((intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORED || intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORING_EMPTY || intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORING) && gamepadInterface1.isKeyDown(RIGHT_BUMPER)) {
                switch (transferState) {
                    case SPEC_WALL_FROM_RUNG:
                    case SPEC_WALL:
                        transferState = RELEASE_FROM_SPEC;
                        break;
                    case RELEASE_FROM_SAMPLE:
                    case RELEASE_FROM_SPEC:
                        transferState = TransferState.BUCKET_TRANSFERRING;
                        break;
                    case BUCKET_TRANSFERRING:
                        transferState = TransferState.RELEASE_FROM_SAMPLE;
                        break;
                }

                FrontalLobe.useMacro(transferState.getMacroAlias());
            }

            if ((intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORED || intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORING_EMPTY || intakeSubsystem.getState() == IntakeSubsystem.IntakeState.STORING) && gamepadInterface1.isKeyDown(B)) {
                switch (transferState) {
                    case RELEASE_FROM_SAMPLE:
                    case RELEASE_FROM_SPEC:
                    case FROM_SUBSTATION:
                        transferState = TransferState.TO_SUBSTATION;
                        break;
                    case TO_SUBSTATION:
                        transferState = TransferState.FROM_SUBSTATION;
                        break;
                }

                FrontalLobe.useMacro(transferState.getMacroAlias());
            }

            if(gamepadInterface1.isKeyDown(DPAD_UP)){
                switch(transferState){
                    case RELEASE_FROM_SAMPLE:
                    case RELEASE_FROM_SPEC:
                    case BUCKET_TRANSFERRING:
                    case TO_SUBSTATION:
                    case FROM_SUBSTATION:
                    case SPEC_WALL:
                    case SPEC_WALL_FROM_RUNG:
                    case HIGH_RUNG:
                        transferState = PISTON;
                        climbSubsystem.setPistonState(ClimbSubsystem.PistonState.UP);
                        linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
                        clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                        armSubsystem.setState(ArmSubsystem.ArmState.CLIMB);
                        climbSubsystem.setPistonState(ClimbSubsystem.PistonState.DOWN);
                        break;
                    case PISTON:
                    case CLIMB_SLIDES_DOWN:
                        climbSubsystem.setPTOState(ClimbSubsystem.PTOState.ENGAGED);
                        slideSubsystem.setState(CLIMB);
                        mecanumController.drive(-1,0,0);
                        transferState = CLIMB_SLIDES_UP;
                        break;
                    case CLIMB_SLIDES_UP:
                        slideSubsystem.climb(true);
                        climbSubsystem.setPTOState(ClimbSubsystem.PTOState.NEUTRAL);
                        slideSubsystem.setState(CLIMB_UP);
                        transferState = CLIMB_SLIDES_DOWN;
                        break;
                }
            }

            if (gamepadInterface1.isKeyDown(TOUCHPAD))
                justRed = !justRed;

            if (justRed)
                gamepad1.setLedColor(255, 0, 0, Integer.MAX_VALUE);
            else
                gamepad1.setLedColor(255, 255, 0, Integer.MAX_VALUE);

            telemetry.addData("transfer state", transferState);
            telemetry.addData("velocity", slideSubsystem.getVelocity());
            telemetry.addData("position", slideSubsystem.getPosition());
            telemetry.update();
            tracker.update();
        }
    }
}
