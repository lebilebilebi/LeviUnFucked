package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;

@Autonomous (name = "blue big 18", group = "AUTON")
public class SolversLibTest extends CommandOpMode {
    private Shooter shooter;
    private Intake intake;
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // init poses
    private final Pose startPose = new Pose(21, 122, Math.toRadians(144));
    private final Pose scorePose = new Pose(58, 74, Math.toRadians(200));
    private final Pose endPose = new Pose(48, 72, Math.toRadians(200));


    // spike 2 intake
    private final Pose spike2IntakePose = new Pose(19, 60);
    private final Pose spike2controlPoint = new Pose(45, 60);

    // gate intake
    private final Pose gateOpenPose = new Pose(17.1, 64);
    private final Pose gateControlPoint = new Pose(40, 64);
    private final Pose gateIntakePose = new Pose(12.7, 59, Math.toRadians(150));

    //spike 1 intake
    private final Pose spike1IntakePose = new Pose(19, 84);
    private final Pose spike1controlPoint = new Pose(43, 83.5);

    // spike 3 intake
    private final Pose spike3IntakePose = new Pose(19, 35.5);
    private final Pose spike3controlPoint = new Pose(52, 36);

    // Path chains
    private PathChain scorePreload, intakeSpike1, intakeSpike2, intakeSpike3, intakeGate, park;
    private PathChain scoreSpike1, scoreSpike2, scoreSpike3, scoreGate;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        //---------------

        // intake and score spike 2
        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, spike2controlPoint, spike2IntakePose))
                .setTangentHeadingInterpolation()
                .build();

        scoreSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(spike2IntakePose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), scorePose.getHeading())
                .build();

        //---------------

        // intake and score gate
        intakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, gateControlPoint, gateOpenPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(gateOpenPose, gateIntakePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), gateIntakePose.getHeading())
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierLine(gateIntakePose, scorePose))
                .setLinearHeadingInterpolation(gateIntakePose.getHeading(), scorePose.getHeading())
                .build();

        //---------------

        // intake and score spike 1
        intakeSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, spike1controlPoint, spike1IntakePose))
                .setTangentHeadingInterpolation()
                .build();

        scoreSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(spike1IntakePose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), scorePose.getHeading())
                .build();

        //---------------

        // intake and score spike 3
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, spike3controlPoint, spike3IntakePose))
                .setTangentHeadingInterpolation()
                .build();

        scoreSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3IntakePose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), scorePose.getHeading())
                .build();

        //---------------

        // park
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    private InstantCommand fire() {
        return new InstantCommand(() -> {
            shooter.requestFireSequence();
        });
    }

    private InstantCommand intake() {
        return new InstantCommand(() -> {
            intake.requestAutoIntake();
        });
    }

    private InstantCommand autoIdle() {
        return new InstantCommand(() -> {
            // Example: hangSubsystem.level1Ascent();
        });
    }

    @Override
    public void initialize() {
        super.reset();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter.setIntake(intake);

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Schedule the autonomous sequence
        schedule(
                new RunCommand(() -> follower.update()),
                new RunCommand(() -> shooter.update()),
                new RunCommand(() -> intake.update()),

                // preload score
                new FollowPathCommand(follower, scorePreload),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // spike 2 intake -> score
                intake(),
                new FollowPathCommand(follower, intakeSpike2),
                new FollowPathCommand(follower, scoreSpike2),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // gate intake -> score (1)
                intake(),
                new FollowPathCommand(follower, intakeGate),
                new FollowPathCommand(follower, scoreGate),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // gate intake -> score (2)
                new FollowPathCommand(follower, intakeGate),
                new FollowPathCommand(follower, scoreGate),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // spike 1 intake -> score
                intake(),
                new FollowPathCommand(follower, intakeSpike1),
                new FollowPathCommand(follower, scoreSpike1),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // spike 3 intake -> score
                intake(),
                new FollowPathCommand(follower, intakeSpike3),
                new FollowPathCommand(follower, scoreSpike3),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // park
                new FollowPathCommand(follower, park, false)
        );
    }

    @Override
    public void run() {
        super.run();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}

