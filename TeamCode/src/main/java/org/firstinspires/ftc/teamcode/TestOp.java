package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@Autonomous(name = "Test OpMode")
public class TestOp extends BaseRobot {
    @Override
    public void runOpMode() {
        super.runOpMode();

        tracker.reset();

        PathContainer path = new PathContainer.PathContainerBuilder()
                .addCurve(new BezierCurve(
                        new Vector2D[]{
                                new Vector2D(0, 0),
                                new Vector2D(0, 10),
                        }
                ))
                .setIncrement(0.1)
                .build();

        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(
                mecanumController,
                tracker,
                path
        )
                .setDeceleration(Double.POSITIVE_INFINITY)
                .setSpeed(1.0)
                .build();

        mecanumController.setDriveSpeed(0.3);

        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        telemetry.update();

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
