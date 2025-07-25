package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

@Disabled
@Autonomous(name = "*** OVERCLOCK - DANGER ***")
public class Overclock extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FrontalLobe.initialize(hardwareMap);
        CortexLinkedMotor bla = MotorCortex.getMotor("bla");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            FrontalLobe.update();

            try {
                bla.setOverclockedPower(0.1);
            } catch (LynxNackException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
