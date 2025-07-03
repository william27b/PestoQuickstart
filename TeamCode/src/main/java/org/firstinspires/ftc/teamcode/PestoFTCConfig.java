package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.Kinematics.DECELERATION;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.END_TOLERANCE_R;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.END_TOLERANCE_XY;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.END_VELOCITY_TOLERANCE;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.SPEED;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.endpointPID;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.headingPID;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.trackers.GoBildaPinpointDriver;
import com.shprobotics.pestocore.drivebases.trackers.GoBildaPinpointTracker;
import com.shprobotics.pestocore.drivebases.trackers.Tracker;
import com.shprobotics.pestocore.drivebases.trackers.TwoWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.processing.Cerebrum;
import com.shprobotics.pestocore.processing.ConfigInterface;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoConfig;

import java.util.function.Function;

@PestoConfig()
public class PestoFTCConfig implements ConfigInterface {
    public static boolean initialized = false;
    public static double ODOMETRY_TICKS_PER_INCH = 505.3169;
    public static double FORWARD_OFFSET = 0;
    public static double ODOMETRY_WIDTH = 14.35782;

    public static class Kinematics {
        public static final double MAX_VELOCITY = 0.0;
        public static final double DECELERATION = 0.0;
    }

    public static class PathFollowing {
        public static final PID endpointPID = new PID(0, 0, 0);
        public static final PID headingPID = new PID(0, 0, 0);

        public static final double INCREMENT = 0.01;
        public static final double SPEED = 0.3;

        public static final double END_VELOCITY_TOLERANCE = 0.0;
        public static final double END_TOLERANCE_XY = 0.0;
        public static final double END_TOLERANCE_R = 0.0;
    }

    public static class Odometry {
        public static final GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public static final GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public static final double encoderResolution = 505.3169;
    }

    public static void initialize(HardwareMap hardwareMap) {
        MotorCortex.initialize(hardwareMap);
        Cerebrum.initialize();

        DriveController driveController = new MecanumController(
                MotorCortex.getMotor("frontLeft"),
                MotorCortex.getMotor("frontRight"),
                MotorCortex.getMotor("backLeft"),
                MotorCortex.getMotor("backRight")
        );

        driveController.configureMotorDirections(new Direction[]{
                Direction.REVERSE,
                Direction.FORWARD,
                Direction.FORWARD,
                Direction.FORWARD
        });

        driveController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        DeterministicTracker tracker = new TwoWheelOdometryTracker.TrackerBuilder(
                hardwareMap,
                ODOMETRY_TICKS_PER_INCH,
                FORWARD_OFFSET,
                ODOMETRY_WIDTH,
                "frontLeft",
                "frontRight",
                Direction.FORWARD,
                Direction.REVERSE
        ).setIMUOrientation(new IMU.Parameters(orientationOnRobot)).build();

//                new GoBildaPinpointTracker.TrackerBuilder(
//                hardwareMap,
//                "pinpoint"
//        )
//                .setXEncoderDirection(Odometry.xDirection)
//                .setYEncoderDirection(Odometry.yDirection)
//                .setEncoderResolution(Odometry.encoderResolution)
//                .build();

//        DeterministicTracker tracker = new ThreeWheelOdometryTracker.TrackerBuilder(
//                hardwareMap,
//                ODOMETRY_TICKS_PER_INCH,
//                FORWARD_OFFSET,
//                ODOMETRY_WIDTH,
//                leftName,
//                centerName,
//                rightName,
//                leftDirection,
//                centerDirection,
//                rightDirection
//        )
//                .build();

        TeleOpController teleOpController = new TeleOpController(driveController, hardwareMap);

        teleOpController.setSpeedController(new Function<Gamepad, Double>() {
            @Override
            public Double apply(Gamepad gamepad) {
                return 1.0;
            }
        });

//        teleOpController.counteractCentripetalForce(tracker, MAX_VELOCITY);

        FrontalLobe.driveController = driveController;
        FrontalLobe.teleOpController = teleOpController;
        FrontalLobe.tracker = tracker;
    }

    public static PathFollower.PathFollowerBuilder createPathFollower(PathContainer pathContainer) {
        return new PathFollower.PathFollowerBuilder(
                FrontalLobe.driveController,
                FrontalLobe.tracker,
                pathContainer
        )
                .setEndpointPID(endpointPID)
                .setHeadingPID(headingPID)
                .setDeceleration(DECELERATION)
                .setSpeed(SPEED)
                .setEndTolerance(END_TOLERANCE_XY, END_TOLERANCE_R)
                .setEndVelocityTolerance(END_VELOCITY_TOLERANCE);
    }
}
