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

        PathContainer path = new PathContainer.PathContainerBuilder()
                .addCurve(new BezierCurve(
                        new Vector2D[]{
                                new Vector2D(0, 0),
                                new Vector2D(0, 10),
                        }
                ))
                .build();

        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(
                mecanumController,
                tracker,
                path
        )
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();
            pathFollower.update();
        }
    }
}
