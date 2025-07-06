package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.DECELERATION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@Autonomous(name = "Test OpMode")
public class TestOp extends BaseRobot {
    @Override
    public void runOpMode() {
        super.runOpMode();

        tracker.reset();

        PathContainer path = new PathContainer.PathContainerBuilder()
                .addCurve(
                        new ParametricHeading(
                                new double[]{
                                        0.0,
                                        Math.PI
                                }
                        )
                )
                .setIncrement(0.1)
                .build();

        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(
                mecanumController,
                tracker,
                path
        )
                .setDeceleration(DECELERATION)
                .setSpeed(1.0)
                .setHeadingPID(new PID(
                        PestoFTCConfig.headingP,
                        PestoFTCConfig.headingI,
                        PestoFTCConfig.headingD
                ))
                .setEndpointPID(new PID(
                        PestoFTCConfig.endpointP,
                        PestoFTCConfig.endpointI,
                        PestoFTCConfig.endpointD
                ))
                .build();

        mecanumController.setDriveSpeed(0.3);

        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        telemetry.update();

        path.reset();
        pathFollower.reset();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();
            pathFollower.update();

            telemetry.addData("x", tracker.getCurrentPosition().getX());
            telemetry.addData("y", tracker.getCurrentPosition().getY());
            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            telemetry.addData("finished", path.isFinished());
            telemetry.addData("completed", pathFollower.isCompleted());
            telemetry.addData("decelerating", pathFollower.isDecelerating());
            telemetry.update();
        }
    }
}
