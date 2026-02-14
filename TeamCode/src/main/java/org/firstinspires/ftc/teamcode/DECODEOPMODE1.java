package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LLSub;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;


@TeleOp(name="TELEOP 1", group="Linear OpMode")
public class DECODEOPMODE1 extends LinearOpMode {

    public enum RobotMode {
        IDLE,
        INTAKING,
        FIRING
    }

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime firingTimer = new ElapsedTime();

    private Intake intake;
    private Shooter shooter;
    private DriveMechanisms drive;
    private LLSub llSub;

    private RobotMode currentMode = RobotMode.IDLE;

    private static final double FIRING_FEED_DELAY = 0.3; // seconds before intake feeds

    @Override
    public void runOpMode() {
        // Initialize subsystems
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setIntake(intake);
        drive = new DriveMechanisms(hardwareMap);
        llSub = new LLSub(this);
        shooter.setLLSub(llSub);
        shooter.setManualTargets(1185, 0.4); //14 and .45

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        llSub.reset();

        while (opModeIsActive()) {
            // Update all subsystems
            llSub.update();

            // Handle input and mode transitions
            handleInput();

            // Update subsystem states based on current mode
            updateRobotMode();

            // Run subsystem updates
            shooter.update();
            intake.update();

            // Drive
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            drive.drive(axial, lateral, yaw);

            // Telemetry
            displayTelemetry();
        }

        llSub.stop();
    }

    private void handleInput() {
        // Cross button - Shoot (spin up and auto-feed)
        if (gamepad2.cross) {
            setMode(RobotMode.FIRING);
        }
        // Right bumper - Intake
        else if (gamepad2.right_bumper) {
            setMode(RobotMode.INTAKING);
        }
        // Left bumper - Stop everything
        else if (gamepad2.left_bumper) {
            setMode(RobotMode.IDLE);
        }
    }

    private void setMode(RobotMode newMode) {
        if (currentMode != newMode) {
            currentMode = newMode;
            firingTimer.reset();

            switch (newMode) {
                case IDLE:
                    shooter.setState(Shooter.ShooterStates.IDLE);
                    intake.setState(Intake.IntakeStates.IDLE);
                    break;

                case INTAKING:
                    shooter.setState(Shooter.ShooterStates.IDLE);
                    intake.setState(Intake.IntakeStates.INTAKE);
                    llSub.startRecentering();
                    break;

                case FIRING:
                    // Open gate immediately; feed after gate-open delay
                    shooter.setState(Shooter.ShooterStates.FIRING);
                    intake.setState(Intake.IntakeStates.IDLE);
                    break;
            }
        }
    }

    private void updateRobotMode() {
        switch (currentMode) {
            case IDLE:
                break;
            case FIRING:
                if (firingTimer.seconds() > FIRING_FEED_DELAY && shooter.isGateOpen()) {
                    intake.setState(Intake.IntakeStates.FEED_SHOOTER);
                } else {
                    intake.setState(Intake.IntakeStates.IDLE);
                }
                break;
        }
    }

    private void displayTelemetry() {
        telemetry.addData("=== MODE ===", currentMode);
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());

        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("State", shooter.getState());
        telemetry.addData("Target Velocity", "%.1f tps", shooter.getTargetVelocity());
        telemetry.addData("Actual Velocity (PRE-COMP)", "%.1f tps", llSub.getCalculatedFlywheelVelPreBattery());
        telemetry.addData("Actual Velocity (POST-COMP)", "%.1f tps", shooter.getCurrentVelocity());
        telemetry.addData("At Speed", shooter.isAtSpeed());
        telemetry.addData("Hood Target", "%.3f", shooter.getTargetHoodPosition());
        telemetry.addData("BatteryComp", llSub.getBatteryCompensation());

        telemetry.addData("Compensation Difference", llSub.getTPSDif());

        telemetry.addData("=== LIMELIGHT ===", "");
        telemetry.addData("Has Target", llSub.hasValidTarget());
        telemetry.addData("Distance", "%.1f in", llSub.getDistanceToGoal());
        telemetry.addData("TX/TY", "%.1f / %.1f", llSub.getTx(), llSub.getTy());

        telemetry.addData("=== INTAKE ===", "");
        telemetry.addData("State", intake.getState());
        telemetry.addData("Has Ball", intake.hasBall());
        telemetry.addData("Current 1/2", "%.2f / %.2f A", intake.getStage1Current(), intake.getStage2Current());

        telemetry.update();
    }
}
