package org.firstinspires.ftc.teamcode.subsystems;

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

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        mecanumController = (MecanumController) FrontalLobe.driveController;
        tracker = FrontalLobe.tracker;
        tracker.reset();
        teleOpController = FrontalLobe.teleOpController;

        clawSubsystem = new ClawSubsystem();
        extendoSubsystem = new ExtendoSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        linkageSubsystem = new LinkageSubsystem();
        slideSubsystem = new SlideSubsystem();
        armSubsystem = new ArmSubsystem(hardwareMap);

        gamepadInterface1 = new GamepadInterface(gamepad1);
        gamepadInterface2 = new GamepadInterface(gamepad2);
    }
}
