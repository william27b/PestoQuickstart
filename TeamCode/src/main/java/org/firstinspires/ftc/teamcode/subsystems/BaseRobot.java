package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem.ExtendoState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.MEDIUM;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.SPEC;
import static org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlideState.UP;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

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

//    public PistonSubsystem pistonSubsystem;
//    public PTOSubsystem ptoSubsystem;

    public GamepadInterface gamepadInterface1;
    public GamepadInterface gamepadInterface2;

    public TransferState transferState;
//    public ClimbState climbState;

    public enum TransferState {
        // NEITHER
        RELEASE_FROM_SAMPLE ("sample - release"),
        RELEASE_FROM_SPEC ("spec - release"),
        AUTO_SPEC_START("auto - spec"),
        AUTO_BUCKET_DEPO ("auto - sample - depo"),

        // SAMPLE
        BUCKET_TRANSFERRING ("sample - bucket"),
        TO_SUBSTATION ("sample - substation"),
        FROM_SUBSTATION ("sample - substation - return"),

        // SPEC
        SPEC_WALL ("spec - wall"),
        SPEC_WALL_FROM_RUNG ("spec - wall - from rung"),
        HIGH_RUNG ("spec - high rung");
        //CLIMB
//        CLIMB_SLIDES_UP ("climb - raise slides"),
//        CLIMB_SLIDES_DOWN("climb - lower slides");


        TransferState(String macroAlias) {
            this.macroAlias = macroAlias;
        }

        private final String macroAlias;

        public String getMacroAlias() {
            return this.macroAlias;
        }
    }

//    public enum ClimbState {
//        // CLIMB
//        NEUTRAL("climb - neutral"),
//        PISTON ("climb - piston"),
//        CLIMB_SLIDES_UP ("climb - raise slides"),
//        CLIMB_SLIDES_DOWN("climb - lower slides");
//        ClimbState(String macroAlias) {
//            this.macroAlias = macroAlias;
//        }
//
//        private final String macroAlias;
//
//        public String getMacroAlias() {
//            return this.macroAlias;
//        }
//    }

    @Override
    public void runOpMode() {
        transferState = TransferState.RELEASE_FROM_SAMPLE;
//        climbState = ClimbState.NEUTRAL;

        FrontalLobe.initialize(hardwareMap);

        mecanumController = (MecanumController) FrontalLobe.driveController;
        if (PestoFTCConfig.initializePinpoint) {
            tracker = FrontalLobe.tracker;
            tracker.reset();

            teleOpController = FrontalLobe.teleOpController;
            teleOpController.configureIMU(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        }

        clawSubsystem = new ClawSubsystem();
        extendoSubsystem = new ExtendoSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        linkageSubsystem = new LinkageSubsystem();
        slideSubsystem = new SlideSubsystem();
        armSubsystem = new ArmSubsystem();

//        ptoSubsystem = new PTOSubsystem();
//        pistonSubsystem = new PistonSubsystem();

        gamepadInterface1 = new GamepadInterface(gamepad1);
        gamepadInterface2 = new GamepadInterface(gamepad2);

        FrontalLobe.addMacro("sample - bucket", new FrontalLobe.Macro() {
            @Override
            public void start() {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.STORED);
                armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
            }

            @Override
            public boolean loop(double v) {

                if (v < 0.5)
                    return false;
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.TRANSFER);

                if (v < 0.6)
                    return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

                if (v < 0.85) // right after the claw closes
                    return false;

                slideSubsystem.setState(UP);

                if (slideSubsystem.getPosition() / (-1350) <= 0.5) // 0.5 is fraction of slides up before pivoting the arm
                    return false;

                armSubsystem.setState(ArmSubsystem.ArmState.BUCKET);

                if (v < 1.85)
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
                if (v > 0.8)
                    return false;

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.INTAKE);
                armSubsystem.setState(ArmSubsystem.ArmState.TRANSFER);
                slideSubsystem.setState(DOWN);

                if (v > 3.0)
                    return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

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
                if (v < 0.1)
                    return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

                if (v < 0.2)
                    return false;

                slideSubsystem.setState(MEDIUM);

                if (v < 0.7)
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

        FrontalLobe.addMacro("spec - wall - from rung", new FrontalLobe.Macro() {
            @Override
            public void start() {
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.WALL);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.2) return false;

                armSubsystem.setState(ArmSubsystem.ArmState.WALL);

                if (v < 0.5) return false;

                slideSubsystem.setState(DOWN);

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
                if (v < 0.1)
                    return false;

                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
                slideSubsystem.setState(SPEC);

                if (v < 1.0)
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
                if (v < 1.0) return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
                slideSubsystem.setState(DOWN);
                armSubsystem.setState(ArmSubsystem.ArmState.WALL);
                slideSubsystem.update();

                return true;
            }
        });

        FrontalLobe.addMacro("auto - spec", new FrontalLobe.Macro() {
            @Override
            public void start() {
                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
            }

            @Override
            public boolean loop(double v) {
                return true;
            }
        });

        FrontalLobe.addMacro("auto - sample - depo", new FrontalLobe.Macro() {
            @Override
            public void start() {
                armSubsystem.setState(ArmSubsystem.ArmState.DEPOSIT);
                linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);
            }

            @Override
            public boolean loop(double v) {
                if (v < 0.6)
                    return false;

                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSED);

                if (v < 0.85) // right after the claw closes
                    return false;

                slideSubsystem.setState(UP);

                if (slideSubsystem.getPosition() / (-1350) <= 0.5) // 0.5 is fraction of slides up before pivoting the arm
                    return false;

                armSubsystem.setState(ArmSubsystem.ArmState.BUCKET);

                if (v < 1.85)
                    return false;

                linkageSubsystem.setState(LinkageSubsystem.LinkageState.OVEREXTENDED);

                return true;
            }
        });

        FrontalLobe.addMacro("slides - reset", new FrontalLobe.Macro() {
            @Override
            public void start() {
                slideSubsystem.topSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideSubsystem.botSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            @Override
            public boolean loop(double v) {
                slideSubsystem.topSlide.setPowerResult(0.6);
                slideSubsystem.botSlide.setPowerResult(0.6);

                return false;
            }
        });

//        FrontalLobe.addMacro("climb - piston", new FrontalLobe.Macro() {
//            @Override
//            public void start() {
//                climbSubsystem.setPistonState(ClimbSubsystem.PistonState.UP);
//                linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
//                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
//            }
//
//            @Override
//            public boolean loop(double v) {
//                if (v < 0.6)
//                    return false;
//
//                armSubsystem.setState(ArmSubsystem.ArmState.CLIMB);
//                climbSubsystem.setPistonState(ClimbSubsystem.PistonState.DOWN);
//
//                return true;
//            }
//        });

//        FrontalLobe.addMacro("climb - raise slides", new FrontalLobe.Macro() {
//            @Override
//            public void start() {
//                climbSubsystem.setPTOState(ClimbSubsystem.PTOState.NEUTRAL);
//            }
//
//            @Override
//            public boolean loop(double v) {
//                if (v < 0.5)
//                    return false;
//
//                slideSubsystem.setState(UP);
//
//                return true;
//            }
//        });

//        FrontalLobe.addMacro("climb - lower slides", new FrontalLobe.Macro() {
//            @Override
//            public void start() {
//                climbSubsystem.setPTOState(ClimbSubsystem.PTOState.ENGAGED);
//            }
//
//            @Override
//            public boolean loop(double v) {
//                if (v < 0.5)
//                    return false;
//
//                slideSubsystem.setState(CLIMB);
//
//                return true;
//            }
//        });


//        FrontalLobe.addMacro("L3 - Piston", new FrontalLobe.Macro() {
//            @Override
//            public void start() {
//                pistonSubsystem.setState(PistonSubsystem.PistonState.UP);
//                linkageSubsystem.setState(LinkageSubsystem.LinkageState.RETRACTED);
//                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
//                armSubsystem.setState(ArmSubsystem.ArmState.CLIMB);
//            }
//
//            @Override
//            public boolean loop(double v) {
//                if (v < 0.8)
//                    return false;
//
//                pistonSubsystem.setState(PistonSubsystem.PistonState.DOWN);
//                ptoSubsystem.setState(NEUTRAL);
//                slideSubsystem.setState(CLIMB_UP);
//
//                return true;
//            }
//        });
    }
}
