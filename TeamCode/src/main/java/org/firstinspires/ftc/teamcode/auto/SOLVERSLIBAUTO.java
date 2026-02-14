package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LLSub;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;

@Autonomous (name = "Redo 12", group = "AUTON")
public class SOLVERSLIBAUTO extends CommandOpMode {
    private Shooter shooter;
    private Intake intake;
    private LLSub llSub;
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // init poses
    private final Pose startPose = new Pose(21, 122, Math.toRadians(144));
    private final Pose scorePose = new Pose(56, 86, Math.toRadians(139));
    private final Pose endPose = new Pose(48, 72, Math.toRadians(200));


    // spike 2 intake
    private final Pose spike2IntakeStartPose = new Pose(55, 65, Math.toRadians(180));
    private final Pose spike2IntakePose = new Pose(30, 65, Math.toRadians(180));
    private final Pose spike2AvoidPose = new Pose(30, 57, Math.toRadians(180));
    //private final Pose spike2controlPoint = new Pose(45, 60);

    // gate intake
    private final Pose gateOpenPose = new Pose(20, 67, Math.toRadians(180));
    private final Pose gateAlignPose = new Pose(35, 67, Math.toRadians(180));

    //private final Pose gateControlPoint = new Pose(40, 64);
    //private final Pose gateIntakePose = new Pose(12.7, 59, Math.toRadians(150));

    //spike 1 intake
    private final Pose spike1IntakePose = new Pose(32, 87, Math.toRadians(180));
    //private final Pose spike1controlPoint = new Pose(43, 83.5);

    // spike 3 intake
    private final Pose spike3IntakeStartPose = new Pose(55, 44, Math.toRadians(180));
    private final Pose spike3IntakePose = new Pose(30, 44, Math.toRadians(180));
    //private final Pose spike3controlPoint = new Pose(52, 36);

    // Path chains
    private PathChain scorePreload, driveToSpike1, intakeSpike1, driveToSpike2, openGate, intakeSpike2, driveToSpike3, intakeSpike3, intakeGate, park;
    private PathChain scoreSpike1, scoreSpike2, scoreSpike3, scoreGate;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        //---------------

        // intake and score spike 2
        driveToSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, spike2IntakeStartPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), spike2IntakeStartPose.getHeading())
                .build();

        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(spike2IntakeStartPose, spike2IntakePose))
                .setLinearHeadingInterpolation(spike2IntakeStartPose.getHeading(), spike2IntakePose.getHeading())

                .addPath(new BezierLine(spike2IntakePose, spike2AvoidPose))
                .setLinearHeadingInterpolation(spike2IntakePose.getHeading(), spike2AvoidPose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(spike2AvoidPose, gateAlignPose))
                .setLinearHeadingInterpolation(spike2AvoidPose.getHeading(), gateAlignPose.getHeading())

                .addPath(new BezierLine(gateAlignPose, gateOpenPose))
                .setLinearHeadingInterpolation(gateAlignPose.getHeading(), gateOpenPose.getHeading())
                .build();

        scoreSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(spike2AvoidPose, scorePose))
                .setLinearHeadingInterpolation(spike2AvoidPose.getHeading(), scorePose.getHeading())
                .build();

        //---------------

        // intake and score gate
        /*intakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, gateControlPoint, gateOpenPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(gateOpenPose, gateIntakePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), gateIntakePose.getHeading())
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierLine(gateIntakePose, scorePose))
                .setLinearHeadingInterpolation(gateIntakePose.getHeading(), scorePose.getHeading())
                .build();*/

        //---------------

        // intake and score spike 1
        intakeSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, spike1IntakePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), spike1IntakePose.getHeading())
                .build();

        scoreSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(spike1IntakePose, scorePose))
                .setLinearHeadingInterpolation(spike1IntakePose.getHeading(), scorePose.getHeading())
                .build();

        //---------------

        // intake and score spike 3
        driveToSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, spike3IntakeStartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), spike3IntakeStartPose.getHeading())
                .build();

        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3IntakeStartPose, spike3IntakePose))
                .setLinearHeadingInterpolation(spike3IntakeStartPose.getHeading(), spike3IntakePose.getHeading())
                .build();

        scoreSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3IntakePose, scorePose))
                .setLinearHeadingInterpolation(spike3IntakePose.getHeading(), scorePose.getHeading())
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
            llSub.startRecentering();
        });
    }

    private InstantCommand autoIdle() {
        return new InstantCommand(() -> {
            shooter.setState(Shooter.ShooterStates.IDLE);
            intake.setState(Intake.IntakeStates.IDLE);
        });
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        llSub = new LLSub(this);
        shooter.setIntake(intake);
        shooter.setLLSub(llSub);
        shooter.setManualTargets(1500, 0.45);

        follower.setStartingPose(startPose);
        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(

                // preload score
                new FollowPathCommand(follower, scorePreload),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // spike 2 intake -> score
                autoIdle(),
                new FollowPathCommand(follower, driveToSpike2),
                intake(),
                new FollowPathCommand(follower, intakeSpike2, 0.3),
                autoIdle(),
                new FollowPathCommand(follower, openGate, 0.3),

                new FollowPathCommand(follower, scoreSpike2),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // spike 1 intake -> score
                new TurnToCommand(follower, 180, AngleUnit.DEGREES),
                new WaitCommand(500), // Wait .5 seconds
                intake(),
                new FollowPathCommand(follower, intakeSpike1, 0.3),
                autoIdle(),
                new FollowPathCommand(follower, scoreSpike1),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // spike 3 intake -> score
                autoIdle(),
                new FollowPathCommand(follower, driveToSpike3),
                intake(),
                new FollowPathCommand(follower, intakeSpike3, 0.3),
                autoIdle(),
                new FollowPathCommand(follower, scoreSpike3),
                fire(),
                new WaitCommand(2000), // Wait 2 seconds

                // park
                autoIdle(),
                new FollowPathCommand(follower, park, false)
        );
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();

        shooter.update();
        intake.update();
        follower.update();
        llSub.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("=== LIMELIGHT ===", "");
        telemetry.addData("Has Target", llSub.hasValidTarget());
        telemetry.addData("Distance", "%.1f in", llSub.getDistanceToGoal());
        telemetry.addData("TX/TY", "%.1f / %.1f", llSub.getTx(), llSub.getTy());
        telemetryData.update();
    }
}
