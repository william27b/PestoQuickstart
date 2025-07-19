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
@Autonomous(name = "***Five Guys***")
public class FiveGuys extends BaseRobot {
    private Follower follower;

    private final Pose startPose = new Pose(7, 66, Math.toRadians(0));

    private final Pose scorePreloadPose = new Pose(28, 74, Math.toRadians(0));

    private final Pose pushOnePose = new Pose(25, 23, Math.toRadians(0));
    private final Pose pushOneControlPoint1 = new Pose(20, 38);
    private final Pose pushOneControlPoint2 = new Pose(30, 5);
    private final Pose pushOneControlPoint3 = new Pose(52, 40);
    private final Pose pushOneControlPoint4 = new Pose(47, 50);
    private final Pose pushOneControlPoint5 = new Pose(76, 12);
    private final Pose pushOneControlPoint6 = new Pose(54, 25);

    private final Pose pushTwoPose = new Pose(25, 15, Math.toRadians(0));
    private final Pose pushTwoControlPoint1 = new Pose(58, 36);
    private final Pose pushTwoControlPoint2 = new Pose(58, 19);
    private final Pose pushTwoControlPoint3 = new Pose(38, 35);
    private final Pose pushTwoControlPoint4 = new Pose(72, 10);
    private final Pose pushTwoControlPoint5 = new Pose(44, 13);

    private final Pose pushThreePose = new Pose(9, 8, Math.toRadians(0));
    private final Pose pushThreeControlPoint1 = new Pose(62, 24);
    private final Pose pushThreeControlPoint2 = new Pose(76, 1);
    private final Pose pushThreeControlPoint3 = new Pose(36, 9);

    private final Pose scoreOnePose = new Pose(34, 72, Math.toRadians(0));
    private final Pose scoreOneControlPoint = new Pose(14, 62);

    private final Pose grabTwoPose = new Pose(7, 35, Math.toRadians(0));
    private final Pose grabTwoControlPoint1 = new Pose(17, 62);
    private final Pose grabTwoControlPoint2 = new Pose(20, 30);

    private final Pose scoreTwoPose = new Pose(34, 70.5, Math.toRadians(0));
    private final Pose scoreTwoControlPoint = new Pose(22, 62);

    private final Pose grabThreePose = new Pose(7, 35, Math.toRadians(0));
    private final Pose grabThreeControlPoint1 = new Pose(17, 62);
    private final Pose grabThreeControlPoint2 = new Pose(20, 30);

    private final Pose scoreThreePose = new Pose(34, 69, Math.toRadians(0));
    private final Pose scoreThreeControlPoint = new Pose(22, 62);

    private final Pose grabFourPose = new Pose(7, 35, Math.toRadians(0));
    private final Pose grabFourControlPoint1 = new Pose(17, 62);
    private final Pose grabFourControlPoint2 = new Pose(20, 30);

    private final Pose scoreFourPose = new Pose(34, 68, Math.toRadians(0));
    private final Pose scoreFourControlPoint = new Pose(22, 62);

    private final Pose grabBucketPose = new Pose(7, 35, Math.toRadians(0));
    private final Pose grabBucketControlPoint1 = new Pose(17, 62);
    private final Pose grabBucketControlPoint2 = new Pose(20, 30);

    private final Pose scoreBucketPose = new Pose(13, 130, Math.toRadians(315));
    private final Pose scoreBucketControlPoint = new Pose(30, 118);

    private Path scorePreload, depositOne,
            grabTwo, depositTwo,
            grabThree, depositThree,
            grabFour, depositFour,
            grabBucket, depositBucket;

    private PathChain pushChain;

    private PathState pathState;

    public enum PathState {
        PRELOAD_TO_SUB,
        SUB_TO_PUSH,
        PUSH_CHAIN,
        DEPOSIT_ONE,
        GRAB_TWO,
        DEPOSIT_TWO,
        GRAB_THREE,
        DEPOSIT_THREE,
        GRAB_FOUR,
        DEPOSIT_FOUR,
        GRAB_BUCKET,
        BUCKET,
        PARK;
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePreloadPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading());

        pushChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreloadPose),
                    new Point(pushOneControlPoint1),
                    new Point(pushOneControlPoint2),
                    new Point(pushOneControlPoint3),
                    new Point(pushOneControlPoint4),
                    new Point(pushOneControlPoint5),
                    new Point(pushOneControlPoint6),
                    new Point(pushOnePose)))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), pushOnePose.getHeading())
                .setPathEndTimeoutConstraint(100)
                .addPath(new BezierCurve(new Point(pushOnePose),
                    new Point(pushTwoControlPoint1),
                    new Point(pushTwoControlPoint2),
                    new Point(pushTwoControlPoint3),
                    new Point(pushTwoControlPoint4),
                    new Point(pushTwoControlPoint5),
                    new Point(pushTwoPose)))
                .setLinearHeadingInterpolation(pushOnePose.getHeading(), pushTwoPose.getHeading())
                .setPathEndTimeoutConstraint(100)
                .addPath(new BezierCurve(new Point(pushTwoPose),
                    new Point(pushThreeControlPoint1),
                    new Point(pushThreeControlPoint2),
                    new Point(pushThreeControlPoint3),
                    new Point(pushThreePose)))
                .setLinearHeadingInterpolation(pushTwoPose.getHeading(), pushThreePose.getHeading())
                .setPathEndTimeoutConstraint(1000)
                .build();

        depositOne = new Path(new BezierCurve(new Point(pushThreePose),
                new Point(scoreOneControlPoint),
                new Point(scoreOnePose)));
        depositOne.setLinearHeadingInterpolation(pushThreePose.getHeading(), scoreOnePose.getHeading());

        grabTwo = new Path(new BezierCurve(new Point(scoreOnePose),
                new Point(grabTwoControlPoint1),
                new Point(grabTwoControlPoint2),
                new Point(grabTwoPose)));
        grabTwo.setLinearHeadingInterpolation(scoreOnePose.getHeading(), grabTwoPose.getHeading());

        depositTwo = new Path(new BezierCurve(new Point(grabTwoPose),
                new Point(scoreTwoControlPoint),
                new Point(scoreTwoPose)));
        depositTwo.setLinearHeadingInterpolation(grabTwoPose.getHeading(), scoreTwoPose.getHeading());

        grabThree = new Path(new BezierCurve(new Point(scoreTwoPose),
                new Point(grabThreeControlPoint1),
                new Point(grabThreeControlPoint2),
                new Point(grabThreePose)));
        grabThree.setLinearHeadingInterpolation(scoreTwoPose.getHeading(), grabThreePose.getHeading());

        depositThree = new Path(new BezierCurve(new Point(grabThreePose),
                new Point(scoreThreeControlPoint),
                new Point(scoreThreePose)));
        depositThree.setLinearHeadingInterpolation(grabThreePose.getHeading(), scoreThreePose.getHeading());

        grabFour = new Path(new BezierCurve(new Point(scoreThreePose),
                new Point(grabFourControlPoint1),
                new Point(grabFourControlPoint2),
                new Point(grabFourPose)));
        grabFour.setLinearHeadingInterpolation(scoreThreePose.getHeading(), grabFourPose.getHeading());

        depositFour = new Path(new BezierCurve(new Point(grabFourPose),
                new Point(scoreFourControlPoint),
                new Point(scoreFourPose)));
        depositFour.setLinearHeadingInterpolation(grabFourPose.getHeading(), scoreFourPose.getHeading());

        grabBucket = new Path(new BezierCurve(new Point(scoreFourPose),
                new Point(grabBucketControlPoint1),
                new Point(grabBucketControlPoint2),
                new Point(grabBucketPose)));
        grabBucket.setLinearHeadingInterpolation(scoreFourPose.getHeading(), grabBucketPose.getHeading());

        depositBucket = new Path(new BezierCurve(new Point(grabBucketPose),
                new Point(scoreBucketControlPoint),
                new Point(scoreBucketPose)));
        depositBucket.setLinearHeadingInterpolation(grabBucketPose.getHeading(), scoreBucketPose.getHeading());

    }

    public void autonomousPathUpdate() {
        slideSubsystem.update();
        switch (pathState) {
            case PRELOAD_TO_SUB:
                follower.followPath(scorePreload);
                break;
            case SUB_TO_PUSH:
                follower.followPath(pushChain);
                break;
            case DEPOSIT_ONE:
                follower.followPath(depositOne);
                break;
            case GRAB_TWO:
                follower.followPath(grabTwo);
                break;
            case DEPOSIT_TWO:
                follower.followPath(depositTwo);
                break;
            case GRAB_THREE:
                follower.followPath(grabThree);
                break;
            case DEPOSIT_THREE:
                follower.followPath(depositThree);
                break;
            case GRAB_FOUR:
                follower.followPath(grabFour);
                break;
            case DEPOSIT_FOUR:
                follower.followPath(depositFour);
                break;
            case GRAB_BUCKET:
                follower.followPath(grabBucket);
                break;
            case BUCKET:
                follower.followPath(depositBucket);
                break;
        }
    }

    public boolean autonomousRobotUpdate(double v) {
        slideSubsystem.update();
        switch (pathState) {
            case PRELOAD_TO_SUB:
                extendoSubsystem.enable();

                slideSubsystem.setState(SPEC); // SPEC-ish

                if (v < 0.1)
                    return false;

                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);

                if (v < 0.9)
                    return false;

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.SPEC);

                if (v < 1.5)
                    return false;

                if (follower.isBusy())
                    return false;

                pathState = PathState.SUB_TO_PUSH;
                return true;
            case SUB_TO_PUSH:

                if (v < 0.5) {
                    clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                    linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);
                    return false;
                }

                slideSubsystem.setState(DOWN);
                armSubsystem.setState(ArmSubsystem.ArmState.WALL);
                slideSubsystem.update();

                if (follower.isBusy() || clawSubsystem.breakbeam.getState())
                    return false;

                if (!FrontalLobe.hasMacro(TransferState.HIGH_RUNG.getMacroAlias()))
                    FrontalLobe.useMacro(TransferState.HIGH_RUNG.getMacroAlias());

                pathState = PathState.DEPOSIT_ONE;
                return true;
            case DEPOSIT_ONE:

                if (follower.isBusy())
                    return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);

                pathState = PathState.GRAB_TWO;
                return true;
            case GRAB_TWO:
                /*prep arm for grab*/

                if (v < 0.5)
                    return false;

                slideSubsystem.setState(DOWN);
                armSubsystem.setState(ArmSubsystem.ArmState.WALL);
                slideSubsystem.update();

                if (follower.isBusy() || clawSubsystem.breakbeam.getState())
                    return false;

                if (!FrontalLobe.hasMacro(TransferState.HIGH_RUNG.getMacroAlias()))
                    FrontalLobe.useMacro(TransferState.HIGH_RUNG.getMacroAlias());

                pathState = PathState.DEPOSIT_TWO;
                return true;
            case DEPOSIT_TWO:
                /*raise arm*/

                if (follower.isBusy())
                    return false;

                if (!FrontalLobe.hasMacro(TransferState.RELEASE_FROM_SPEC.getMacroAlias()))
                    FrontalLobe.useMacro(TransferState.RELEASE_FROM_SPEC.getMacroAlias());
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);

                pathState = PathState.GRAB_THREE;
                return true;
            case GRAB_THREE:
                /*prep arm for grab*/
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                if (follower.isBusy())
                    return false;

                if (!FrontalLobe.hasMacro(TransferState.HIGH_RUNG.getMacroAlias()))
                    FrontalLobe.useMacro(TransferState.HIGH_RUNG.getMacroAlias());

                pathState = PathState.DEPOSIT_THREE;
                return true;
            case DEPOSIT_THREE:
                /*raise arm*/
                if (follower.isBusy())
                    return false;

                if (!FrontalLobe.hasMacro(TransferState.RELEASE_FROM_SPEC.getMacroAlias()))
                    FrontalLobe.useMacro(TransferState.RELEASE_FROM_SPEC.getMacroAlias());
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);

                pathState = PathState.GRAB_FOUR;
                return true;
            case GRAB_FOUR:
                /*prep arm for grab*/
                if (follower.isBusy())
                    return false;

                if (!FrontalLobe.hasMacro(TransferState.HIGH_RUNG.getMacroAlias()))
                    FrontalLobe.useMacro(TransferState.HIGH_RUNG.getMacroAlias());

                pathState = PathState.DEPOSIT_FOUR;
                return true;
            case DEPOSIT_FOUR:
                /*raise arm*/
                if (follower.isBusy())
                    return false;

                if (!FrontalLobe.hasMacro(TransferState.RELEASE_FROM_SPEC.getMacroAlias()))
                    FrontalLobe.useMacro(TransferState.RELEASE_FROM_SPEC.getMacroAlias());
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);

                pathState = PathState.GRAB_BUCKET;
                return true;
            case GRAB_BUCKET:
                /*prep arm for grab*/
                if (follower.isBusy())
                    return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

                pathState = PathState.BUCKET;
                return true;
            case BUCKET:
                /*raise arm*/
                if(follower.getPose().getY() > 72)
                    if (!FrontalLobe.hasMacro(TransferState.AUTO_BUCKET_DEPO.getMacroAlias()))
                        FrontalLobe.useMacro(TransferState.AUTO_BUCKET_DEPO.getMacroAlias());

                if (follower.isBusy())
                    return false;

                if (!FrontalLobe.hasMacro(TransferState.RELEASE_FROM_SAMPLE.getMacroAlias()))
                    FrontalLobe.useMacro(TransferState.RELEASE_FROM_SAMPLE.getMacroAlias());

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
        follower.setStartingPose(startPose);
        buildPaths();

        pathState = PathState.PRELOAD_TO_SUB;

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
            telemetry.update();
        }
    }
}

