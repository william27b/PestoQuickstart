package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.Kinematics.DECELERATION;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.END_TOLERANCE_R;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.END_TOLERANCE_XY;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.END_VELOCITY_TOLERANCE;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.SPEED;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.endpointPID;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.PathFollowing.headingPID;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.processing.Cerebrum;
import com.shprobotics.pestocore.processing.ConfigInterface;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoConfig;

@PestoConfig()
public class PestoFTCConfig implements ConfigInterface {
    public static boolean initialized = false;

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
        public static final double ODOMETRY_TICKS_PER_INCH = 505.316944316;
        public static final double FORWARD_OFFSET = -6.5;
        public static final double ODOMETRY_WIDTH = 14;

        public static final String leftName = "frontLeft";
        public static final String centerName = "frontRight";
        public static final String rightName = "backRight";

        public static final Direction leftDirection = Direction.FORWARD;
        public static final Direction centerDirection = Direction.FORWARD;
        public static final Direction rightDirection = Direction.FORWARD;
    }

    public static void initialize(HardwareMap hardwareMap) {
        MotorCortex.initialize(hardwareMap);
        Cerebrum.initialize();

        DriveController driveController = new MecanumController(
                MotorCortex.getMotor("0"),
                MotorCortex.getMotor("1"),
                MotorCortex.getMotor("2"),
                MotorCortex.getMotor("3")
        );

//        driveController.configureMotorDirections(new Direction[]{
//                Direction.REVERSE,
//                Direction.FORWARD,
//                Direction.REVERSE,
//                Direction.FORWARD
//        });
//
//        driveController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
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
//
//        TeleOpController teleOpController = new TeleOpController(driveController, hardwareMap);

//        teleOpController.setSpeedController(new Function<Gamepad, Double>() {
//            @Override
//            public Double apply(Gamepad gamepad) {
//                return 0.0;
//            }
//        });

//        teleOpController.counteractCentripetalForce(tracker, MAX_VELOCITY);

        FrontalLobe.driveController = driveController;
//        FrontalLobe.teleOpController = teleOpController;
//        FrontalLobe.tracker = tracker;
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
