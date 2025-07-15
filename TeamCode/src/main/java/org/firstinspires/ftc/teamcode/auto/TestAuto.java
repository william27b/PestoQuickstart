package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.subsystems.BaseRobot.SpecState.HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem.ExtendoState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.MEDIUM;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.SPEC;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.UP;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "**Test Auto***")
public class TestAuto extends OpMode {
    protected ClawSubsystem clawSubsystem;
    protected ExtendoSubsystem extendoSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected LinkageSubsystem linkageSubsystem;
    protected SlideSubsystem slideSubsystem;
    protected ArmSubsystem armSubsystem;

    protected BaseRobot.SpecState specState;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(8, 72, Math.toRadians(270));
    private final Pose scorePose = new Pose(40, 72, Math.toRadians(315));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain grabPickup1;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /**
         * this is a pathchain example
         */
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();
    }

    public void buildMacros(){
        FrontalLobe.initialize(hardwareMap);

        FrontalLobe.addMacro("sample - bucket", new FrontalLobe.Macro() {
            @Override
            public void start() {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORED);
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.25) // right after the claw closes
                    return false;

                slideSubsystem.setState(UP);

                if (slideSubsystem.getPosition() / (-1350) <= 0.5) // 0.5 is fraction of slides up before pivoting the arm
                    return false;

                armSubsystem.setState(ArmSubsystem.ArmState.BUCKET);

                if (v < 1.25)
                    return false;

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);

                return true;
            }
        });

        FrontalLobe.addMacro("sample - release", new FrontalLobe.Macro() {
            @Override
            public void start() {
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                FrontalLobe.removeMacros("sample");
            }

            @Override
            public boolean loop(double v) {
                if (v > 0.5)
                    return false;

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.INTAKE);
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
                armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
                slideSubsystem.setState(DOWN);

                return true;
            }
        });

        FrontalLobe.addMacro("sample - substation", new FrontalLobe.Macro() {
            @Override
            public void start() {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORED);
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
                slideSubsystem.setState(MEDIUM);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.5) return false;

                armSubsystem.setState(ArmSubsystem.ArmState.WALL);
                slideSubsystem.setState(DOWN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);

                return true;
            }
        });

        FrontalLobe.addMacro("sample - substation - return", new FrontalLobe.Macro() {
            @Override
            public void start() {
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                slideSubsystem.setState(MEDIUM);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.5) return false;

                armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
                slideSubsystem.setState(DOWN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.INTAKE);

                return true;
            }
        });

        FrontalLobe.addMacro("intake - store", new FrontalLobe.Macro() {
            @Override
            public void start() {
                extendoSubsystem.setState(IN);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORING);
            }

            @Override
            public boolean loop(double v) {
                return true;
            }
        });


        FrontalLobe.addMacro("spec - wall", new FrontalLobe.Macro() {
            @Override
            public void start() {
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                slideSubsystem.setState(MEDIUM);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.5) return false;

                armSubsystem.setState(ArmSubsystem.ArmState.WALL);
                slideSubsystem.setState(DOWN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);

                return true;
            }
        });

        FrontalLobe.addMacro("spec - high rung", new FrontalLobe.Macro() {
            @Override
            public void start() {
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.2)
                    return false;

                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
                slideSubsystem.setState(SPEC);

                if (v < 0.7)
                    return false;

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);

                return true;
            }
        });

        FrontalLobe.addMacro("spec - release", new FrontalLobe.Macro() {
            @Override
            public void start() {
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.INTAKE);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.5) return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
                slideSubsystem.setState(DOWN);
                armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);

                return true;
            }
        });
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                specState = HIGH_RUNG;
                FrontalLobe.removeMacros("spec");
                FrontalLobe.useMacro(specState.getMacroAlias());
                setPathState(1);
                break;
            case 1:
                follower.followPath(scorePreload);
                break;
            case 2:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(3);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        MotorCortex.update();
        FrontalLobe.update();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        buildMacros();

        specState = BaseRobot.SpecState.TO_WALL;
//        FrontalLobe.removeMacros("spec");
//        FrontalLobe.useMacro(specState.getMacroAlias());
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

//        extendoSubsystem.enable();
//        slideSubsystem.enable();

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

