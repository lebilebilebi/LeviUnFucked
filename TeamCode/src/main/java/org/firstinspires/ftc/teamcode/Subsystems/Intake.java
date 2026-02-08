package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    public enum IntakeStates {
        IDLE,
        INTAKE,
        OUTTAKE,
        FEED_SHOOTER  // New state for feeding balls to shooter
    }

    private DcMotorEx intakeStage1 = null;
    private DcMotorEx intakeStage2 = null;
    private IntakeStates state = IntakeStates.IDLE;

    private double stage1Current = 0;
    private double stage2Current = 0;
    private boolean stage1Stopped = false;
    private boolean stage2Stopped = false;
    private ElapsedTime intakeGraceTimer = new ElapsedTime();

    private static final double STAGE_ONE_CURRENT_LIMIT = 5.6;
    private static final double STAGE_TWO_CURRENT_LIMIT = 5.5;
    private static final double INTAKE_GRACE_PERIOD = 0.3;

    public Intake(HardwareMap hardwareMap) {
        intakeStage1 = hardwareMap.get(DcMotorEx.class, "int1");
        intakeStage2 = hardwareMap.get(DcMotorEx.class, "int2");

        intakeStage1.setDirection(DcMotorEx.Direction.FORWARD);
        intakeStage2.setDirection(DcMotorEx.Direction.REVERSE);

        intakeStage1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeStage2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setState(IntakeStates newState) {
        this.state = newState;
        stage1Stopped = false;
        stage2Stopped = false;
        intakeGraceTimer.reset();
    }

    public IntakeStates getState() {
        return state;
    }

    public double getStage1Current() {
        return stage1Current;
    }

    public double getStage2Current() {
        return stage2Current;
    }

    public boolean isStage1Stopped() {
        return stage1Stopped;
    }

    public boolean isStage2Stopped() {
        return stage2Stopped;
    }

    public boolean hasBall() {
        return stage2Stopped;
    }

    public void update() {
        stage1Current = intakeStage1.getCurrent(CurrentUnit.AMPS);
        stage2Current = intakeStage2.getCurrent(CurrentUnit.AMPS);

        switch (state) {
            case IDLE:
                intakeStage1.setPower(0);
                intakeStage2.setPower(0);
                break;

            case INTAKE:
                if (intakeGraceTimer.seconds() > INTAKE_GRACE_PERIOD) {
                    if (stage1Current >= STAGE_ONE_CURRENT_LIMIT) {
                        stage1Stopped = true;
                    }
                    if (stage2Current >= STAGE_TWO_CURRENT_LIMIT) {
                        stage2Stopped = true;
                    }
                }

                if (stage1Stopped) {
                    intakeStage1.setPower(0);
                } else {
                    intakeStage1.setPower(1);
                }

                if (stage2Stopped) {
                    intakeStage2.setPower(0);
                } else {
                    intakeStage2.setPower(1);
                }
                break;

            case OUTTAKE:
                intakeStage1.setPower(-1);
                intakeStage2.setPower(-1);
                break;

            case FEED_SHOOTER:
                // Run both stages at full power to feed balls into shooter
                // No current limiting - we want to push balls through
                intakeStage1.setPower(1);
                intakeStage2.setPower(1);
                break;
        }
    }
}
