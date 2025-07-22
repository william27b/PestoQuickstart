package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.SPEC;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

@Config
@Autonomous(name = "***Slide Debugging***")
public class SlideDebugging extends BaseRobot {
    private Follower follower;

    PathState pathState;
    public enum PathState {
        RAISE,
        LOWER;
    }

    private final Pose startPose = new Pose(30, 15, Math.toRadians(0));

    private final Pose pushTwoPose = new Pose(55, 9, Math.toRadians(0));

    private final Pose pushTwoControlPoint = new Pose(56, 18, Math.toRadians(0));
    private final Pose pushThreePose = new Pose(28, 3);
    private final Pose pushThreeControlPoint = new Pose(53, 1);
    private PathChain pushChain;

    public void buildPaths() {
        pushChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose),
                        new Point(pushTwoControlPoint),
                        new Point(pushTwoPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pushTwoPose.getHeading())
                .addPath(new BezierCurve(new Point(pushTwoPose),
                        new Point(pushThreeControlPoint),
                        new Point(pushThreePose)))
                .setLinearHeadingInterpolation(pushTwoPose.getHeading(), pushThreePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        slideSubsystem.update();
        switch (pathState) {
            case RAISE:
                follower.followPath(pushChain);
                break;
            case LOWER:

                break;

        }
    }

    public boolean autonomousRobotUpdate(double v) {
        slideSubsystem.update();
        switch (pathState) {
            case RAISE:
//                extendoSubsystem.enable();
//
//                clawSubsystem.setState(ClawSubsystem.ClawState.LOOSE);
//
//                if (v<0.5) return false;
//
//                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
//
//                pathState = PathState.LOWER;
                return true;
            case LOWER:

//                if (v < 0.5) {
//                    clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
//                    linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);
//                    return false;
//                }

//                armSubsystem.setState(ArmSubsystem.ArmState.WALL);
//                slideSubsystem.update();

                return true;

        }

        return false;
    }

    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = false;

        super.runOpMode();

        transferState = TransferState.AUTO_SPEC_START;
        FrontalLobe.removeMacros("spec");
        FrontalLobe.useMacro(transferState.getMacroAlias());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        buildPaths();

        pathState = PathState.RAISE;

        waitForStart();

        double whenChangePath = System.nanoTime();
        autonomousPathUpdate();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            FrontalLobe.update();
            slideSubsystem.update();

            follower.update();

            boolean pathSegmentFinished = autonomousRobotUpdate((System.nanoTime() - whenChangePath) / 1E9);
            if (pathSegmentFinished) {
                autonomousPathUpdate();
                whenChangePath = System.nanoTime();
            }

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Slide velocity", slideSubsystem.getVelocity());
            telemetry.addData("motor power", slideSubsystem.getCurrentPower());

            telemetry.update();
        }
    }
}

