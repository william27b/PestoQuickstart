package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;

@Autonomous(name = "*** 5+0 Pathing ***")
public class FiveGuysPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(7, 66, Math.toRadians(0));

    private final Pose scorePreloadPose = new Pose(34, 66, Math.toRadians(0));

    private final Pose pushOnePose = new Pose(16, 25, Math.toRadians(0));
    private final Pose pushOneControlPoint1 = new Pose(20, 38);
    private final Pose pushOneControlPoint2 = new Pose(35, 32);
    private final Pose pushOneControlPoint3 = new Pose(52, 40);
    private final Pose pushOneControlPoint4 = new Pose(47, 50);
    private final Pose pushOneControlPoint5 = new Pose(76, 12);
    private final Pose pushOneControlPoint6 = new Pose(54, 30);

    private final Pose pushTwoPose = new Pose(19, 15, Math.toRadians(0));
    private final Pose pushTwoControlPoint1 = new Pose(58, 36);
    private final Pose pushTwoControlPoint2 = new Pose(70, 20);
    private final Pose pushTwoControlPoint3 = new Pose(38, 35);
    private final Pose pushTwoControlPoint4 = new Pose(84, 11);
    private final Pose pushTwoControlPoint5 = new Pose(44, 13);

    private final Pose pushThreePose = new Pose(9, 8, Math.toRadians(0));
    private final Pose pushThreeControlPoint1 = new Pose(62, 24);
    private final Pose pushThreeControlPoint2 = new Pose(76, 1);
    private final Pose pushThreeControlPoint3 = new Pose(36, 9);

    private final Pose scoreOnePose = new Pose(34, 66, Math.toRadians(0));
    private final Pose scoreOneControlPoint = new Pose(14, 62);

    private final Pose grabTwoPose = new Pose(7, 32, Math.toRadians(0));
    private final Pose grabTwoControlPoint1 = new Pose(17, 62);
    private final Pose grabTwoControlPoint2 = new Pose(20, 30);

    private final Pose scoreTwoPose = new Pose(34, 66, Math.toRadians(0));
    private final Pose scoreTwoControlPoint = new Pose(22, 62);

    private final Pose grabThreePose = new Pose(7, 32, Math.toRadians(0));
    private final Pose grabThreeControlPoint1 = new Pose(17, 62);
    private final Pose grabThreeControlPoint2 = new Pose(20, 30);

    private final Pose scoreThreePose = new Pose(34, 66, Math.toRadians(0));
    private final Pose scoreThreeControlPoint = new Pose(22, 62);

    private final Pose grabFourPose = new Pose(7, 32, Math.toRadians(0));
    private final Pose grabFourControlPoint1 = new Pose(17, 62);
    private final Pose grabFourControlPoint2 = new Pose(20, 30);

    private final Pose scoreFourPose = new Pose(34, 66, Math.toRadians(0));
    private final Pose scoreFourControlPoint = new Pose(22, 62);

    private final Pose grabBucketPose = new Pose(7, 32, Math.toRadians(0));
    private final Pose grabBucketControlPoint1 = new Pose(17, 62);
    private final Pose grabBucketControlPoint2 = new Pose(20, 30);

    private final Pose scoreBucketPose = new Pose(13, 130, Math.toRadians(315));
    private final Pose scoreBucketControlPoint = new Pose(30, 118);

    private Path scorePreload, pushOne, pushTwo, pushThree, depositOne,
            grabTwo, depositTwo,
            grabThree, depositThree,
            grabFour, depositFour,
            grabBucket, depositBucket;
    private PathState pathState;

    public enum PathState {
        PRELOAD_TO_SUB,
        SUB_TO_PUSH,
        PUSH_TWO,
        PUSH_THREE,
        //GRAB_ONE,
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

        pushOne = new Path(new BezierCurve(new Point(scorePreloadPose),
                new Point(pushOneControlPoint1),
                new Point(pushOneControlPoint2),
                new Point(pushOneControlPoint3),
                new Point(pushOneControlPoint4),
                new Point(pushOneControlPoint5),
                new Point(pushOneControlPoint6),
                new Point(pushOnePose)));
        pushOne.setLinearHeadingInterpolation(scorePreloadPose.getHeading(), pushOnePose.getHeading());

        pushTwo = new Path(new BezierCurve(new Point(pushOnePose),
                new Point(pushTwoControlPoint1),
                new Point(pushTwoControlPoint2),
                new Point(pushTwoControlPoint3),
                new Point(pushTwoControlPoint4),
                new Point(pushTwoControlPoint5),
                new Point(pushTwoPose)));
        pushTwo.setLinearHeadingInterpolation(pushOnePose.getHeading(), pushTwoPose.getHeading());

        pushThree = new Path(new BezierCurve(new Point(pushTwoPose),
                new Point(pushThreeControlPoint1),
                new Point(pushThreeControlPoint2),
                new Point(pushThreeControlPoint3),
                new Point(pushThreePose)));
        pushThree.setLinearHeadingInterpolation(pushTwoPose.getHeading(), pushThreePose.getHeading());

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
        switch (pathState) {
            case PRELOAD_TO_SUB:
                /*include arm up*/

                follower.followPath(scorePreload);
                setPathState(PathState.SUB_TO_PUSH);
                break;
            case SUB_TO_PUSH:
                if(!follower.isBusy()) {
                    /*release spec*/

                    follower.followPath(pushOne);
                    setPathState(PathState.PUSH_TWO);
                }
                break;
            case PUSH_TWO:
                if(!follower.isBusy()) {
                    follower.followPath(pushTwo);
                    setPathState(PathState.PUSH_THREE);
                }
                break;
            case PUSH_THREE:
                if(!follower.isBusy()) {
                    follower.followPath(pushThree);
                    setPathState(PathState.DEPOSIT_ONE);
                    /*maybe add grabOne state*/
                }
                break;
            case DEPOSIT_ONE:
                if(!follower.isBusy()) {
                    /*raise arm*/
                    follower.followPath(depositOne);
                    setPathState(PathState.GRAB_TWO);
                }
                break;

            case GRAB_TWO:
                if(!follower.isBusy()) {
                    /*prep arm for grab*/
                    follower.followPath(grabTwo);
                    setPathState(PathState.DEPOSIT_TWO);
                }
                break;

            case DEPOSIT_TWO:
                if(!follower.isBusy()) {
                    /*raise arm*/
                    follower.followPath(depositTwo);
                    setPathState(PathState.GRAB_THREE);
                }
                break;

            case GRAB_THREE:
                if(!follower.isBusy()) {
                    /*prep arm for grab*/
                    follower.followPath(grabThree);
                    setPathState(PathState.DEPOSIT_THREE);
                }
                break;

            case DEPOSIT_THREE:
                if(!follower.isBusy()) {
                    /*raise arm*/
                    follower.followPath(depositThree);
                    setPathState(PathState.GRAB_FOUR);
                }
                break;

            case GRAB_FOUR:
                if(!follower.isBusy()) {
                    /*prep arm for grab*/
                    follower.followPath(grabFour);
                    setPathState(PathState.DEPOSIT_FOUR);
                }
                break;

            case DEPOSIT_FOUR:
                if(!follower.isBusy()) {
                    /*raise arm*/
                    follower.followPath(depositFour);
                    setPathState(PathState.GRAB_BUCKET);
                }
                break;

            case GRAB_BUCKET:
                if(!follower.isBusy()) {
                    /*prep arm for grab*/
                    follower.followPath(grabBucket);
                    setPathState(PathState.BUCKET);
                }
                break;

            case BUCKET:
                if(!follower.isBusy()) {
                    /*raise arm*/
                    follower.followPath(depositBucket);
                    setPathState(PathState.PARK);
                }
                break;

        }
    }

    public void setPathState(PathState pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(PathState.PRELOAD_TO_SUB);
    }

    @Override
    public void stop() {
    }
}

