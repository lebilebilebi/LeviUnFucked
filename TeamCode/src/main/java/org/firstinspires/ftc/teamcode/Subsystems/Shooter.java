package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    public enum ShooterStates {
        IDLE,
        SPIN_UP,
        READY_TO_FIRE,
        FIRING
    }

    // Hardware
    public DcMotorEx shootR = null;
    public DcMotorEx shootL = null;
    private Servo gate = null;
    private Servo rgbLight = null;
    private Servo hoodR = null;
    private Servo hoodL = null;

    // State
    private ShooterStates state = ShooterStates.IDLE;
    private double currentVelocity = 0;
    private double targetVelocity = 0;
    private double targetHoodPosition = 0;
    private boolean isAtSpeed = false;

    // Timers
    private final ElapsedTime ledTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Constants
    private static final double GATE_OPEN = 0.64;
    private static final double GATE_CLOSED = 0.2;
    private static final double VELOCITY_THRESHOLD = 0.95; // 95% of target
    private static final double SPIN_UP_TIMEOUT = 2.0; // seconds
    private static final double LED_CYCLE_DURATION = 0.7;
    private static final double DEFAULT_TARGET_VELOCITY = 1500.0;
    private static final double DEFAULT_HOOD_POSITION = 0.45;
    private static final double GATE_OPEN_TIME = 0.3;
5    private static final double FEED_TIME = 1.5;

    // Reference to LLSub for auto-targeting
    private LLSub llSub;
    private Intake intake; // optional link for auto-feed

    private enum FireSequenceStage { NONE, SPINNING_UP, GATE_OPENING, FEEDING }
    private FireSequenceStage fireStage = FireSequenceStage.NONE;
    private final ElapsedTime fireTimer = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap) {
        shootR = hardwareMap.get(DcMotorEx.class, "shootR");
        shootL = hardwareMap.get(DcMotorEx.class, "shootL");
        gate = hardwareMap.get(Servo.class, "gate");
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");
        hoodR = hardwareMap.get(Servo.class, "hoodR");
        hoodL = hardwareMap.get(Servo.class, "hoodL");

        shootR.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(19.8, 0, 0, 12.5960);
        shootR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Initialize to safe positions
        gate.setPosition(GATE_CLOSED);
        rgbLight.setPosition(0);
        ledTimer.reset();
    }

    public void setLLSub(LLSub llSub) {
        this.llSub = llSub;
    }

    public void setIntake(Intake intake) {
        this.intake = intake;
    }

    public void setState(ShooterStates newState) {
        this.state = newState;
        stateTimer.reset();
        gateTimer.reset();

        if (newState == ShooterStates.IDLE) {
            gate.setPosition(GATE_CLOSED);
            isAtSpeed = false;
        } else if (newState == ShooterStates.FIRING) {
            openGate(); // open immediately on entry
        }
    }

    public ShooterStates getState() {
        return state;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetHoodPosition() {
        return targetHoodPosition;
    }

    public boolean isAtSpeed() {
        return isAtSpeed;
    }

    public boolean isReadyToFire() {
        return state == ShooterStates.READY_TO_FIRE || state == ShooterStates.FIRING;
    }

    public void setManualTargets(double velocity, double hoodPosition) {
        this.targetVelocity = velocity;
        this.targetHoodPosition = hoodPosition;
    }

    private void setFlywheelVelocity(double velocity) {
        shootR.setVelocity(velocity);
        shootL.setPower(shootR.getPower());
    }

    private void setHoodPosition(double position) {
        hoodR.setPosition(position);
        hoodL.setPosition(position);
    }

    private void updateRainbowLED() {
        double time = ledTimer.seconds();
        double t = (time % LED_CYCLE_DURATION) / LED_CYCLE_DURATION;
        double minPos = 0.280;
        double maxPos = 0.722;
        double rainbowPosition = minPos + (t * (maxPos - minPos));
        rgbLight.setPosition(rainbowPosition);
    }

    private void updateTargetsFromLLOrFallback() {
        if (llSub != null && llSub.hasValidTarget() && llSub.getFinalFlywheelVel() > 0) {
            targetVelocity = llSub.getFinalFlywheelVel();
            targetHoodPosition = llSub.getFinalHoodAngle();
        }
        if (targetVelocity <= 0) {
            targetVelocity = DEFAULT_TARGET_VELOCITY;
            targetHoodPosition = DEFAULT_HOOD_POSITION;
        }
    }

    public void openGate() {
        gate.setPosition(GATE_OPEN);
        gateTimer.reset();
    }

    public void closeGate() {
        gate.setPosition(GATE_CLOSED);
    }

    public boolean isGateOpen() {
        return gateTimer.seconds() >= GATE_OPEN_TIME;
    }

    public void requestFireSequence() {
        fireStage = FireSequenceStage.SPINNING_UP;
        fireTimer.reset();
        setState(ShooterStates.SPIN_UP);
    }

    public void update() {
        currentVelocity = shootR.getVelocity();

        switch (state) {
            case IDLE:
                updateTargetsFromLLOrFallback();
                setFlywheelVelocity(targetVelocity);
                setHoodPosition(targetHoodPosition);
                closeGate();
                isAtSpeed = targetVelocity > 0 && currentVelocity >= targetVelocity * VELOCITY_THRESHOLD;
                updateRainbowLED();
                break;

            case SPIN_UP:
                updateTargetsFromLLOrFallback();

                if (targetVelocity <= 0) {
                    updateRainbowLED();
                    break;
                }

                setFlywheelVelocity(targetVelocity);
                setHoodPosition(targetHoodPosition);
                closeGate();

                if (currentVelocity >= targetVelocity * VELOCITY_THRESHOLD) {
                    isAtSpeed = true;
                    setState(ShooterStates.READY_TO_FIRE);
                } else {
                    isAtSpeed = false;
                    updateRainbowLED();
                }

                if (stateTimer.seconds() > SPIN_UP_TIMEOUT) {
                    setState(ShooterStates.READY_TO_FIRE);
                }
                break;

            case READY_TO_FIRE:
                updateTargetsFromLLOrFallback();
                setFlywheelVelocity(targetVelocity);
                setHoodPosition(targetHoodPosition);
                rgbLight.setPosition(0.5);
                isAtSpeed = currentVelocity >= targetVelocity * VELOCITY_THRESHOLD;
                break;

            case FIRING:
                updateTargetsFromLLOrFallback();
                setFlywheelVelocity(targetVelocity);
                setHoodPosition(targetHoodPosition);
                rgbLight.setPosition(0.5);
                // gate opened on entry; keep spinning/aiming
                break;
        }

        updateFireSequence();
    }

    private void updateFireSequence() {
        if (fireStage == FireSequenceStage.NONE || intake == null) return;

        switch (fireStage) {
            case SPINNING_UP:
                if (targetVelocity > 0 && currentVelocity >= targetVelocity * VELOCITY_THRESHOLD) {
                    openGate();
                    setState(ShooterStates.FIRING);
                    fireStage = FireSequenceStage.GATE_OPENING;
                    fireTimer.reset();
                }
                break;

            case GATE_OPENING:
                if (isGateOpen()) {
                    intake.setState(Intake.IntakeStates.FEED_SHOOTER);
                    fireStage = FireSequenceStage.FEEDING;
                    fireTimer.reset();
                }
                break;

            case FEEDING:
                if (fireTimer.seconds() >= FEED_TIME) {
                    intake.setState(Intake.IntakeStates.IDLE);
                    closeGate();
                    setState(ShooterStates.IDLE);
                    fireStage = FireSequenceStage.NONE;
                }
                break;
        }
    }
}
