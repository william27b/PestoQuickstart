package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.subsystems.PTOSubsystem.PTOState.DISENGAGED;
import static org.firstinspires.ftc.teamcode.subsystems.PTOSubsystem.PTOState.ENGAGED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.PTOSubsystem;

@Autonomous
public class TestOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FrontalLobe.initialize(hardwareMap);
        PTOSubsystem ptoSubsystem = new PTOSubsystem();
        GamepadInterface gamepadInterface = new GamepadInterface(gamepad1);

        waitForStart();

        ptoSubsystem.setState(ENGAGED);

        while (opModeIsActive() && !isStopRequested()) {
            gamepadInterface.update();
            MotorCortex.update();
            FrontalLobe.update();

            if (gamepadInterface.isKeyDown(GamepadKey.DPAD_UP))
                ptoSubsystem.setState((ptoSubsystem.getState() == ENGAGED) ? DISENGAGED : ENGAGED);
        }
    }
}
