package org.firstinspires.ftc.teamcode.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Triangle Test", group = "PIDF Tuning")
public class TriangleTest extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 20;

    private int segment = 0;

    private Follower follower;

    private Path segmentA, segmentB, segmentC;


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        segmentA = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        segmentA.setConstantHeadingInterpolation(0);
        segmentB = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(DISTANCE/1.5,DISTANCE, Point.CARTESIAN)));
        segmentB.setConstantHeadingInterpolation(0);
        segmentC = new Path(new BezierLine(new Point(DISTANCE/1.5,DISTANCE, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        segmentC.setConstantHeadingInterpolation(0);

        follower.followPath(segmentA);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (segment%3 == 0) {
                segment++;
                follower.followPath(segmentA);
            } else if (segment%3 == 1) {
                segment++;
                follower.followPath(segmentB);
            }else {
                segment++;
                follower.followPath(segmentC);
            }
        }
        follower.telemetryDebug(telemetryA);
    }
}
