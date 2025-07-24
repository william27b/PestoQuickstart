package org.firstinspires.ftc.teamcode.auto;

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
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PistonSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

@Autonomous(name = "*** Four Girls ***")
public class FourGirls extends OpMode {

    public ClawSubsystem clawSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public LinkageSubsystem linkageSubsystem;
    public SlideSubsystem slideSubsystem;
    public ArmSubsystem armSubsystem;
    public PistonSubsystem pistonSubsystem;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private ElapsedTime elapsedTime;

    private final Pose startPose = new Pose(7, 115, Math.toRadians(270));
    private final Pose bucketPose = new Pose(20, 122, Math.toRadians(315));

    private final Pose sampleOnePose = new Pose(24, 120, Math.toRadians(0));
    private final Pose sampleTwoPose = new Pose(24, 130, Math.toRadians(0));
    private final Pose sampleThreePose = new Pose(45, 112, Math.toRadians(90));
    private final Pose parkPose = new Pose(60, 100, Math.toRadians(0));


    private Path scorePreload,
            grabOne, depositOne,
            grabTwo, depositTwo,
            grabThree, depositThree,
            park;

    private PathState pathState;

    public enum PathState {
        PRELOAD_TO_BUCKET,
        GRAB_ONE,
        DEPOSIT_ONE,
        GRAB_TWO,
        DEPOSIT_TWO,
        GRAB_THREE,
        DEPOSIT_THREE,
        PARK;
    }
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose),
                new Point(bucketPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), bucketPose.getHeading());

        grabOne = new Path(new BezierLine(new Point(bucketPose),
                new Point(sampleOnePose)));
        grabOne.setLinearHeadingInterpolation(bucketPose.getHeading(), sampleOnePose.getHeading());

        depositOne = new Path(new BezierCurve(new Point(sampleOnePose),
                new Point(bucketPose)));
        depositOne.setLinearHeadingInterpolation(sampleOnePose.getHeading(), bucketPose.getHeading());

        grabTwo = new Path(new BezierCurve(new Point(bucketPose),
                new Point(sampleTwoPose)));
        grabTwo.setLinearHeadingInterpolation(bucketPose.getHeading(), sampleTwoPose.getHeading());

        depositTwo = new Path(new BezierCurve(new Point(sampleTwoPose),
                new Point(bucketPose)));
        depositTwo.setLinearHeadingInterpolation(sampleTwoPose.getHeading(), bucketPose.getHeading());

        grabThree = new Path(new BezierCurve(new Point(bucketPose),
                new Point(sampleThreePose)));
        grabThree.setLinearHeadingInterpolation(bucketPose.getHeading(), sampleThreePose.getHeading());

        depositThree = new Path(new BezierCurve(new Point(sampleThreePose),
                new Point(bucketPose)));
        depositThree.setLinearHeadingInterpolation(sampleThreePose.getHeading(), bucketPose.getHeading());

        park = new Path(new BezierCurve(new Point(bucketPose),
                new Point(parkPose)));
        park.setLinearHeadingInterpolation(bucketPose.getHeading(), parkPose.getHeading());


    }
    public void autonomousPathUpdate() {
        if (follower.isBusy() && clawSubsystem.breakbeam.getState())
            return;

        switch (pathState) {
            case PRELOAD_TO_BUCKET:
                follower.followPath(scorePreload);

//                armSubsystem.setState(ArmSubsystem.ArmState.BUCKET);
//                slideSubsystem.setState(UP);
//
//                wait(1.0);
//                linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);

                setPathState(PathState.GRAB_ONE);
                break;

            case GRAB_ONE:
//                grabSample();

                follower.followPath(grabOne);

//                if(intakeSubsystem.getSample().equals("yellow"))
                    setPathState(PathState.DEPOSIT_ONE);

                break;

            case DEPOSIT_ONE:
//                depositSample();
                follower.followPath(depositOne);

                break;

            case GRAB_TWO:
//                grabSample();
                follower.followPath(grabTwo);

//                if(intakeSubsystem.getSample().equals("yellow"))
                    setPathState(PathState.DEPOSIT_ONE);

                break;

            case DEPOSIT_TWO:
//                depositSample();
                follower.followPath(depositTwo);

                setPathState(PathState.GRAB_THREE);
                break;

            case GRAB_THREE:
//                grabSample();
                follower.followPath(grabThree);

//                if(intakeSubsystem.getSample().equals("yellow"))
                    setPathState(PathState.DEPOSIT_ONE);

                break;

            case DEPOSIT_THREE:
//                depositSample();
                follower.followPath(depositThree);

                setPathState(PathState.PARK);
                break;

            case PARK:
//                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
//
//                armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
//                linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
//                slideSubsystem.setState(DOWN);

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
        armSubsystem = new ArmSubsystem();
        pistonSubsystem = new PistonSubsystem();

        pistonSubsystem.deactivate();
        extendoSubsystem.enable();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
        linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
        clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        extendoSubsystem.enable();
        opmodeTimer.resetTimer();
        setPathState(PathState.PRELOAD_TO_BUCKET);

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

    public void grabSample(){
        clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);

        armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
        linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
        slideSubsystem.setState(DOWN);

        extendoSubsystem.setState(ExtendoSubsystem.ExtendoState.OUT);
        intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
    }

    public void depositSample(){
        intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORING);
        extendoSubsystem.setState(ExtendoSubsystem.ExtendoState.IN);

        clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
        linkageSubsystem.setState(LinkageSubsystem.LinkageState.INTAKE);
        wait(1.0);

        clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
        wait(0.5);

        armSubsystem.setState(ArmSubsystem.ArmState.BUCKET);
        slideSubsystem.setState(UP);

        wait(1.0);
        linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);
    }
}

