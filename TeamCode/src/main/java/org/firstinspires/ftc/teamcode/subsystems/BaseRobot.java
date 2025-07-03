package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem.ExtendoState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.UP;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;

public class BaseRobot extends LinearOpMode {
    protected MecanumController mecanumController;
    protected DeterministicTracker tracker;
    protected TeleOpController teleOpController;

    protected ClawSubsystem clawSubsystem;
    protected ExtendoSubsystem extendoSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected LinkageSubsystem linkageSubsystem;
    protected SlideSubsystem slideSubsystem;
    protected ArmSubsystem armSubsystem;

    protected GamepadInterface gamepadInterface1;
    protected GamepadInterface gamepadInterface2;

    protected TransferState transferState;

    public enum TransferState {
        TRANSFERRING ("sample - grab"),
        RELEASING ("sample - release"),
        RETURNING ("sample - return");

        TransferState(String macroAlias) {
            this.macroAlias = macroAlias;
        }

        private final String macroAlias;

        public String getMacroAlias() {
            return this.macroAlias;
        }
    }

    @Override
    public void runOpMode() {
        transferState = TransferState.RETURNING;

        FrontalLobe.initialize(hardwareMap);

        mecanumController = (MecanumController) FrontalLobe.driveController;
        tracker = FrontalLobe.tracker;
        tracker.reset();
        teleOpController = FrontalLobe.teleOpController;

        teleOpController.configureIMU(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

        clawSubsystem = new ClawSubsystem();
        extendoSubsystem = new ExtendoSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        linkageSubsystem = new LinkageSubsystem();
        slideSubsystem = new SlideSubsystem();
        armSubsystem = new ArmSubsystem(hardwareMap);

        gamepadInterface1 = new GamepadInterface(gamepad1);
        gamepadInterface2 = new GamepadInterface(gamepad2);

        FrontalLobe.addMacro("sample - grab", new FrontalLobe.Macro() {
            @Override
            public void start() {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORED);
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.30) // this is a wait for the transfer before the transfer continues
                    return false;

                slideSubsystem.setState(UP);
                armSubsystem.setState(ArmSubsystem.ArmState.BUCKET);

                return true;
            }
        });

        FrontalLobe.addMacro("sample - release", new FrontalLobe.Macro() {
            @Override
            public void start() {
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
            }

            @Override
            public boolean loop(double v) {
                return true;
            }
        });

        FrontalLobe.addMacro("sample - return", new FrontalLobe.Macro() {
            @Override
            public void start() {
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
                armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
                slideSubsystem.setState(DOWN);
            }

            @Override
            public boolean loop(double v) {
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
    }
}
