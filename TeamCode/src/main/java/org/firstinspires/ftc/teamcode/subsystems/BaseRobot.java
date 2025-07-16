package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem.ExtendoState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.MEDIUM;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.SPEC;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.UP;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;

public class BaseRobot extends LinearOpMode {
    public MecanumController mecanumController;
    public DeterministicTracker tracker;
    public TeleOpController teleOpController;

    public ClawSubsystem clawSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public LinkageSubsystem linkageSubsystem;
    public SlideSubsystem slideSubsystem;
    public ArmSubsystem armSubsystem;

    public GamepadInterface gamepadInterface1;
    public GamepadInterface gamepadInterface2;

    public TransferState transferState;

    public enum TransferState {
        // NEITHER
        RELEASE_FROM_SAMPLE ("sample - release"),
        RELEASE_FROM_SPEC ("spec - release"),

        // SAMPLE
        BUCKET_TRANSFERRING ("sample - bucket"),
        TO_SUBSTATION ("sample - substation"),
        FROM_SUBSTATION ("sample - substation - return"),

        // SPEC
        SPEC_WALL ("spec - wall"),
        HIGH_RUNG ("spec - high rung");

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
        transferState = TransferState.RELEASE_FROM_SAMPLE;

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

        FrontalLobe.addMacro("sample - bucket", new FrontalLobe.Macro() {
            @Override
            public void start() {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORED);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.TRANSFER);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.1)
                    return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

                if (v < 0.35) // right after the claw closes
                    return false;

                slideSubsystem.setState(UP);

                if (slideSubsystem.getPosition() / (-1350) <= 0.5) // 0.5 is fraction of slides up before pivoting the arm
                    return false;

                armSubsystem.setState(ArmSubsystem.ArmState.BUCKET);

                if (v < 1.35)
                    return false;

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);

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
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.TRANSFER);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.1) //TODO: Check timing
                    return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

                if (v < 0.6)
                    return false;

                slideSubsystem.setState(MEDIUM);

                if (v < 1.6)
                    return false;

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

        FrontalLobe.addMacro("intake - outtake", new FrontalLobe.Macro() {
            @Override
            public void start() {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.TO_OUTTAKE);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.2)
                    return false;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTTAKE);

                return false;
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
                if (v < 0.1) //changed from 0.2 to 0.1 for issue #12
                    return false;

                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
                slideSubsystem.setState(SPEC);

                if (v < 0.6) //from 0.7 to 0.6 for issue #12
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
                armSubsystem.setState(ArmSubsystem.ArmState.WALL); //from transfer to wall for issue #12

                return true;
            }
        });
    }
}
