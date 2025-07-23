package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.AUTO_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.SPEC;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.UP;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

@Autonomous(name = "*** 5+1 ***")
public class FiveGuysPathing extends OpMode {

    public ClawSubsystem clawSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public LinkageSubsystem linkageSubsystem;
    public SlideSubsystem slideSubsystem;
    public ArmSubsystem armSubsystem;
    public ClimbSubsystem climbSubsystem;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private ElapsedTime elapsedTime;

    private final Pose startPose = new Pose(7, 66, Math.toRadians(0));

    private final Pose scorePreloadPose = new Pose(28, 74, Math.toRadians(0));

    private final Pose pushOnePose = new Pose(27.5, 22.5, Math.toRadians(0));
    private final Pose pushOneControlPoint1 = new Pose(12, 40);
    private final Pose pushOneControlPoint2 = new Pose(35, 14);
    private final Pose pushOneControlPoint3 = new Pose(51, 33);
    private final Pose pushOneControlPoint4 = new Pose(47, 50);
    private final Pose pushOneControlPoint5 = new Pose(66, 12);
    private final Pose pushOneControlPoint6 = new Pose(54, 24);

    private final Pose pushTwoPose = new Pose(27.5, 15, Math.toRadians(0));
    private final Pose pushTwoControlPoint1 = new Pose(58, 36);
    private final Pose pushTwoControlPoint2 = new Pose(65, 15);
    private final Pose pushTwoControlPoint3 = new Pose(38, 35);
    private final Pose pushTwoControlPoint4 = new Pose(77, 10);
    private final Pose pushTwoControlPoint5 = new Pose(44, 13);

    private final Pose pushThreePose = new Pose(10.5, 7, Math.toRadians(0));
    private final Pose pushThreeControlPoint1 = new Pose(62, 24);
    private final Pose pushThreeControlPoint2 = new Pose(70, 1);
    private final Pose pushThreeControlPoint3 = new Pose(36, 7);

    private final Pose scoreOnePose = new Pose(28, 72.5, Math.toRadians(0));
    private final Pose scoreOneControlPoint = new Pose(7, 70);

    private final Pose grabTwoPose = new Pose(15, 38, Math.toRadians(0));
    private final Pose grabTwoControlPoint1 = new Pose(17, 62);
    private final Pose grabTwoControlPoint2 = new Pose(20, 40);

    private final Pose scoreTwoPose = new Pose(28, 71, Math.toRadians(0));
    private final Pose scoreTwoControlPoint = new Pose(7, 70);

    private final Pose grabThreePose = new Pose(15, 38, Math.toRadians(0));
    private final Pose grabThreeControlPoint1 = new Pose(17, 62);
    private final Pose grabThreeControlPoint2 = new Pose(20, 40);

    private final Pose scoreThreePose = new Pose(28, 69, Math.toRadians(0));
    private final Pose scoreThreeControlPoint = new Pose(7, 70);

    private final Pose grabFourPose = new Pose(15, 38, Math.toRadians(0)); // TODO change 15 to 14.5
    private final Pose grabFourControlPoint1 = new Pose(17, 62);
    private final Pose grabFourControlPoint2 = new Pose(20, 40);

    private final Pose scoreFourPose = new Pose(28, 67, Math.toRadians(0));
    private final Pose scoreFourControlPoint = new Pose(7, 70);

    private final Pose grabBucketPose = new Pose(11, 60, Math.toRadians(270)); //TODO Check that robot isn't killing itself into wall

    private final Pose scoreBucketPose = new Pose(25, 120, Math.toRadians(315));
    private final Pose scoreBucketControlPoint = new Pose(30, 118);

    private final Pose parkPose = new Pose(20, 120, Math.toRadians(0));
    private Path scorePreload, depositOne,
            grabTwo, depositTwo,
            grabThree, depositThree,
            grabFour, depositFour,
            grabBucket, depositBucket, park;

    private PathChain pushChain;
    private PathState pathState;

    public enum PathState {
        PRELOAD_TO_SUB,
        SUB_TO_PUSH,
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
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTimeoutConstraint(100)
                .addPath(new BezierCurve(new Point(pushOnePose),
                        new Point(pushTwoControlPoint1),
                        new Point(pushTwoControlPoint2),
                        new Point(pushTwoControlPoint3),
                        new Point(pushTwoControlPoint4),
                        new Point(pushTwoControlPoint5),
                        new Point(pushTwoPose)))
                .setLinearHeadingInterpolation(pushOnePose.getHeading(), pushTwoPose.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTimeoutConstraint(100)
                .addPath(new BezierCurve(new Point(pushTwoPose),
                        new Point(pushThreeControlPoint1),
                        new Point(pushThreeControlPoint2),
                        new Point(pushThreeControlPoint3),
                        new Point(pushThreePose)))
                .setLinearHeadingInterpolation(pushTwoPose.getHeading(), pushThreePose.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTimeoutConstraint(100)
                .build();

        depositOne = new Path(new BezierCurve(new Point(pushThreePose),
                    new Point(scoreOneControlPoint),
                    new Point(scoreOnePose)));
        depositOne.setLinearHeadingInterpolation(pushThreePose.getHeading(), scoreOnePose.getHeading());
        depositOne.setZeroPowerAccelerationMultiplier(4);


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
                new Point(grabBucketPose)));
        grabBucket.setLinearHeadingInterpolation(scoreFourPose.getHeading(), grabBucketPose.getHeading());

        depositBucket = new Path(new BezierCurve(new Point(grabBucketPose),
                new Point(scoreBucketControlPoint),
                new Point(scoreBucketPose)));
        depositBucket.setLinearHeadingInterpolation(grabBucketPose.getHeading(), scoreBucketPose.getHeading());

        park = new Path(new BezierCurve(new Point(scoreBucketPose),
                new Point(parkPose)));
        park.setLinearHeadingInterpolation(scoreBucketPose.getHeading(), parkPose.getHeading());


    }
    public void autonomousPathUpdate() {
        if (follower.isBusy() && clawSubsystem.breakbeam.getState())
            return;

        switch (pathState) {
            case PRELOAD_TO_SUB:
                follower.followPath(scorePreload);

                slideSubsystem.climb(true);
                slideSubsystem.setState(SPEC);

                wait(0.1);
                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);

                wait(0.5);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.SPEC);

                setPathState(PathState.SUB_TO_PUSH);
                break;

            case SUB_TO_PUSH:
                slideSubsystem.climb(false);

                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);

                wait(0.1);

                follower.followPath(pushChain);

                wait(0.5);

                armSubsystem.setState(ArmSubsystem.ArmState.WALL);
                slideSubsystem.setState(AUTO_DOWN);
                slideSubsystem.update();

                wait(0.5);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.AUTO);

                setPathState(PathState.DEPOSIT_ONE);

                break;

            case DEPOSIT_ONE:
                grabSpec();

                follower.followPath(depositOne);

                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
                slideSubsystem.setState(SPEC);
                setPathState(PathState.GRAB_TWO);
                break;

            case GRAB_TWO:
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);

                follower.followPath(grabTwo);

                wait(0.25);
                slideSubsystem.setState(AUTO_DOWN);
                armSubsystem.setState(ArmSubsystem.ArmState.WALL);

                if(!clawSubsystem.breakbeam.getState()) {
                    setPathState(PathState.DEPOSIT_TWO);
                }

                break;

            case DEPOSIT_TWO:
                grabSpec();

                follower.followPath(depositTwo);

                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
                slideSubsystem.setState(SPEC);
                wait(1.0);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);

                setPathState(PathState.GRAB_THREE);
                break;

            case GRAB_THREE:
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);

                follower.followPath(grabThree);

                wait(0.25);
                slideSubsystem.setState(AUTO_DOWN);
                armSubsystem.setState(ArmSubsystem.ArmState.WALL);

                if(!clawSubsystem.breakbeam.getState()) {
                    setPathState(PathState.DEPOSIT_THREE);
                }
                break;

            case DEPOSIT_THREE:
                grabSpec();

                follower.followPath(depositThree);

                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
                slideSubsystem.setState(SPEC);
                wait(1.0);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);

                setPathState(PathState.GRAB_FOUR);
                break;

            case GRAB_FOUR:
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);

                follower.followPath(grabFour);

                wait(0.5);
                slideSubsystem.setState(AUTO_DOWN);
                armSubsystem.setState(ArmSubsystem.ArmState.WALL);

                if(!clawSubsystem.breakbeam.getState()) {
                    setPathState(PathState.DEPOSIT_FOUR);
                }
                break;

            case DEPOSIT_FOUR:
                grabSpec();

                follower.followPath(depositFour);

                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
                slideSubsystem.setState(SPEC);
                wait(1.0);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);

                setPathState(PathState.GRAB_BUCKET);
                break;

            case GRAB_BUCKET:
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);

                follower.followPath(grabBucket);

                wait(0.5);
                slideSubsystem.setState(AUTO_DOWN);
                armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.TRANSFER);

                extendoSubsystem.setState(ExtendoSubsystem.ExtendoState.OUT);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);

                if(!intakeSubsystem.getSample().equals("nothing")) {
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORING);
                    extendoSubsystem.setState(ExtendoSubsystem.ExtendoState.IN);

                    setPathState(PathState.BUCKET);
                }

                break;

            case BUCKET:
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                follower.followPath(depositBucket);
                wait(0.5);

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.INTAKE);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORED);
                wait(0.5);
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

                wait(0.25);

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.TRANSFER);
                wait(0.5);

                slideSubsystem.setState(UP);
                wait(1.0);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);
                armSubsystem.setState(ArmSubsystem.ArmState.BUCKET);

                setPathState(PathState.PARK);
                break;

            case PARK:
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                wait(0.5);
                follower.followPath(park);

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.INTAKE);
                armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
                slideSubsystem.setState(DOWN);
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
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
        updateSubsystems();

        FrontalLobe.update();
        MotorCortex.update();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        FrontalLobe.initialize(hardwareMap);
        MotorCortex.initialize(hardwareMap);

        clawSubsystem = new ClawSubsystem();
        extendoSubsystem = new ExtendoSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        linkageSubsystem = new LinkageSubsystem();
        slideSubsystem = new SlideSubsystem();
        armSubsystem = new ArmSubsystem(hardwareMap);
        climbSubsystem = new ClimbSubsystem();

        extendoSubsystem.enable();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
        linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
        clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        extendoSubsystem.enable();
        opmodeTimer.resetTimer();
        setPathState(PathState.PRELOAD_TO_SUB);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    @Override
    public void stop() {
    }

    public void updateSubsystems(){
        follower.update();
        clawSubsystem.update();
        extendoSubsystem.update();
        intakeSubsystem.update();
        linkageSubsystem.update();
        slideSubsystem.update();
        armSubsystem.update();
    }

    public void wait(double sec){
        elapsedTime.reset();
        while (elapsedTime.seconds() < sec){
            follower.update();
        }
    }

    public void grabSpec(){
        clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
        wait(0.25);
    }

    public void grabBucket(){
        clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
        wait(0.25);

        slideSubsystem.setState(UP);
        armSubsystem.setState(ArmSubsystem.ArmState.BUCKET);
        linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);
    }
}

