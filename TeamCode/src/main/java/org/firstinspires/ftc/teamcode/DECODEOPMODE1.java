package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LLSub;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;

import java.util.List;

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
    private Follower follower; // Added Follower
    private List<LynxModule> allHubs;

    private RobotMode currentMode = RobotMode.IDLE;

    private static final double FIRING_FEED_DELAY = 0.3;

    @Override
    public void runOpMode() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize subsystems
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setIntake(intake);
        drive = new DriveMechanisms(hardwareMap);
        llSub = new LLSub(this);

        // --- NEW: Initialize Follower for TeleOp ---
        // This is required for field-centric tracking to work
        follower = Constants.createFollower(hardwareMap);
        // Important: Set starting pose to (0,0,0) or carry over from Auto if possible
        follower.setStartingPose(PoseStorage.lastPose);

        shooter.setLLSub(llSub);
        shooter.setManualTargets(1500, 0.45);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        // llSub.reset(); // Removed: method no longer exists

        while (opModeIsActive()) {
            // Update Follower FIRST (gets odometry)
            follower.update();

            // Update Limelight Subsystem (consumes odometry + relocalizes)
            llSub.update(follower);

            // Handle input and mode transitions
            handleInput();

            // Update subsystem states based on current mode
            updateRobotMode();

            // Run subsystem updates
            shooter.update();
            intake.update();

            // Drive Controls
            // Note: If you want to use PedroPathing's field centric drive, utilize follower.setTeleOpMovementVectors()
            // But keeping your existing drive logic is fine too, as long as follower.update() is called.
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
        if (gamepad2.cross) {
            setMode(RobotMode.FIRING);
        } else if (gamepad2.right_bumper) {
            setMode(RobotMode.INTAKING);
        } else if (gamepad2.left_bumper) {
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
                    break;
                case FIRING:
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
        telemetry.addData("Actual Velocity", "%.1f tps", shooter.getCurrentVelocity());
        telemetry.addData("Hood Target", "%.3f", shooter.getTargetHoodPosition());

        telemetry.addData("=== LIMELIGHT ===", "");
        telemetry.addData("Has Target", llSub.hasValidTarget());
        telemetry.addData("Distance", "%.1f in", llSub.getDistanceToGoal());
        // Tx/Ty removed because we are doing field-centric calculations now
        telemetry.addData("Turret Angle", "%.2f", llSub.getFinalHoodAngle()); // Or add a getTurretAngle() if needed
        telemetry.addData("Turret kP", "%.4f", LLSub.turretKp);

        telemetry.addData("=== INTAKE ===", "");
        telemetry.addData("State", intake.getState());

        telemetry.update();
    }
}