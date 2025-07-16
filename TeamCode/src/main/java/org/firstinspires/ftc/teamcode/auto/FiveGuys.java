package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@Autonomous(name = "**Five Guys***")
public class FiveGuys extends BaseRobot {
    private Follower follower;
    
    private final Pose startPose = new Pose(8, 72, Math.toRadians(270));
    private final Pose scorePose = new Pose(40, 72, Math.toRadians(315));

    private PathState pathState;

    public enum PathState {
        PRELOAD_TO_SUB,
        SUB_TO_PUSH;
    }

    private Path scorePreload;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD_TO_SUB:
                transferState = TransferState.HIGH_RUNG;
                FrontalLobe.removeMacros("spec");
                FrontalLobe.useMacro(transferState.getMacroAlias());

                pathState = PathState.SUB_TO_PUSH;
                break;
            case SUB_TO_PUSH:
                follower.followPath(scorePreload);
                break;
        }
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        pathState = PathState.PRELOAD_TO_SUB;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            FrontalLobe.update();

            follower.update();
            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}

