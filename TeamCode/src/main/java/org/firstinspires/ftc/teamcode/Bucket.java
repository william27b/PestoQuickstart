package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.DECELERATION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

@Autonomous(name = "Bucket")
public class Bucket extends BaseRobot {
    static PathContainer score = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(0, 0),
                                    new Vector2D(0, 27.75)
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer first_right = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(0, 27.75),
                                    new Vector2D(34.25, 27.75),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer first_up = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(34.25, 27.75),
                                    new Vector2D(34.25, 50.25),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer second_right = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(34.25, 50.25),
                                    new Vector2D(39.65, 50.25),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer first_down = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(39.65, 50.25),
                                    new Vector2D(39.65, 10.75),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer second_up = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(39.65, 10.75),
                                    new Vector2D(39.65, 50.25),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer third_right = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(39.65, 50.25),
                                    new Vector2D(50.15, 50.25),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer second_down = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(50.15, 50.25),
                                    new Vector2D(50.15, 10.75),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer third_up = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(50.15, 10.75),
                                    new Vector2D(50.15, 50.25),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer fourth_right = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(50.15, 50.25),
                                    new Vector2D(57.15, 50.25),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    static PathContainer third_down = new PathContainer.PathContainerBuilder()
            .addCurve(
                    new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(57.15, 50.25),
                                    new Vector2D(57.15, 5.00),
                            }
                    )
            )
            .setIncrement(0.1)
            .build();

    @FunctionalInterface
    public interface PathAction {
        void run(BaseRobot baseRobot);
    }
    
    public enum AutoState {
        SCORE (score, (baseRobot) -> {}),
        FIRST_RIGHT (first_right, (baseRobot) -> {}),
        FIRST_UP (first_up, (baseRobot) -> {}),
        SECOND_RIGHT (second_right, (baseRobot) -> {}),
        FIRST_DOWN (first_down, (baseRobot) -> {}),
        SECOND_UP (second_up, (baseRobot) -> {}),
        THIRD_RIGHT (third_right, (baseRobot) -> {}),
        SECOND_DOWN (second_down, (baseRobot) -> {}),
        THIRD_UP (third_up, (baseRobot) -> {}),
        FOURTH_RIGHT (fourth_right, (baseRobot) -> {}),
        THIRD_DOWN (third_down, (baseRobot) -> {});

        AutoState(PathContainer path, PathAction action) {
            this.path = path;
            this.action = action;
        }

        public PathFollower getPathFollower(MecanumController mecanumController, DeterministicTracker tracker) {
            return new PathFollower.PathFollowerBuilder(
                    mecanumController,
                    tracker,
                    this.path
            )
                    .setDeceleration(DECELERATION)
                    .setSpeed(0.6)
                    .setHeadingPID(new PID(
                            PestoFTCConfig.headingP,
                            PestoFTCConfig.headingI,
                            PestoFTCConfig.headingD
                    ))
                    .setEndpointPID(new PID(
                            PestoFTCConfig.endpointP,
                            PestoFTCConfig.endpointI,
                            PestoFTCConfig.endpointD
                    ))
                    .build();
        }

        private PathContainer path;
        private PathAction action;
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

        AutoState autoState = AutoState.SCORE;
        PathFollower pathFollower = autoState.getPathFollower(mecanumController, tracker);

        tracker.reset();

        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        telemetry.update();

        pathFollower.reset();

        clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

        waitForStart();
        double started = System.nanoTime();

        slideSubsystem.setState(SlideSubsystem.SlideState.SPEC);
        sleep(1000);
        armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
        linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();
            pathFollower.update();

            if (pathFollower.isFinished(0.5, 0.2) || ((System.nanoTime() - started) / 1E9) > 2.0) {
                autoState.action.run(this);
                
                started = System.nanoTime();
                switch (autoState) {
                    case SCORE:
                        autoState = AutoState.FIRST_RIGHT;
                        break;
//                    case FIRST_RIGHT:
//                        autoState = AutoState.FIRST_UP;
//                        break;
//                    case FIRST_UP:
//                        autoState = AutoState.SECOND_RIGHT;
//                        break;
//                    case SECOND_RIGHT:
//                        autoState = AutoState.FIRST_DOWN;
//                        break;
//                    case FIRST_DOWN:
//                        autoState = AutoState.SECOND_UP;
//                        break;
//                    case SECOND_UP:
//                        autoState = AutoState.THIRD_RIGHT;
//                        break;
//                    case THIRD_RIGHT:
//                        autoState = AutoState.SECOND_DOWN;
//                        break;
//                    case SECOND_DOWN:
//                        autoState = AutoState.THIRD_UP;
//                        break;
//                    case THIRD_UP:
//                        autoState = AutoState.FOURTH_RIGHT;
//                        break;
//                    case FOURTH_RIGHT:
//                        autoState = AutoState.THIRD_DOWN;
//                        break;
                    default:
                        return;
                }

                pathFollower = autoState.getPathFollower(mecanumController, tracker);
                pathFollower.reset();
            }

            telemetry.addData("x", tracker.getCurrentPosition().getX());
            telemetry.addData("y", tracker.getCurrentPosition().getY());
            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            telemetry.addData("finished", autoState.path.isFinished());
            telemetry.addData("completed", pathFollower.isCompleted());
            telemetry.addData("decelerating", pathFollower.isDecelerating());
            telemetry.update();
        }
    }
}
