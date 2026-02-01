package org.firstinspires.ftc.teamcode.Subsystems;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class InternalMechanisms {
    public enum RoboStates {
        IDLE,
        SPIN_UP,
        INTAKE,
        FIRST_BALL_STOP,
        FULL_STOP,
        SHOOT,
        FAR_SIDE,
        CLOSE_SIDE,
        FULL_IDLE,
        AUTO_SCORE,
    }
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorEx shootR = null;
    public DcMotorEx shootL = null;
    private DcMotorEx intakeStage1 = null;
    private DcMotorEx intakeStage2 = null;
    private Servo gate = null;
    private Servo rgbLight;
    private Servo hoodR = null;
    private Servo hoodL = null;
    private RoboStates IOState = RoboStates.IDLE;
    private double currentVelocity = 0;
    private double flywheelVelocity = 0;
    private double hoodPosition = 0;
    double stage1Current = 0;
    double stage2Current = 0;
    double cycleDuration = 0.7;
    private int shotsRemaining = 0;
    private double goalDist = 0;
    private double goalX = 0;
    private double goalY = 0;
    public boolean shotsFired = false;
    private final ElapsedTime ledTimer = new ElapsedTime();
    public ElapsedTime gateTimer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean gateOpened = false;
    private boolean stage1Stopped = false;
    private boolean stage2Stopped = false;
    private ElapsedTime intakeGraceTimer = new ElapsedTime(); // Timer for intake startup grace period

    private static final double STAGE_ONE_CURRENT_LIMIT = 5.6;
    private static final double STAGE_TWO_CURRENT_LIMIT = 5.5;
    private static final double INTAKE_GRACE_PERIOD = 0.3; // Adjustable grace period in seconds
    private static final double MAX_FLYWHEEL_VELOCITY = 2200;
    private static final double MIN_FLYWHEEL_VELOCITY = 0;
    private static final double MAX_HOOD_POSITION = 0.9;
    private static final double MIN_HOOD_POSITION = 0;
    private double GATE_OPEN= 0.64;
    private double GATE_CLOSED= 0.2;
    private double GATE_OPEN_TIME= 0.18;
    private double GATE_CLOSE_TIME = 0.18;
    private static final double AUTO_SCORE_VELOCITY_THRESHOLD = 1510; // Add threshold constant
    private static final double FAR_AUTO_SCORE_VELOCITY_THRESHOLD = 2100; // Add threshold constant

    public static double getHoodAngle(double goalDist) {
        double rawAngle = (0.00000120563 * Math.pow(goalDist, 3))
                - (0.000235615 * Math.pow(goalDist, 2))
                + (0.00615079 * goalDist)
                + 0.57;
        return MathFunctions.clamp(rawAngle, 0.0, 0.9);
    }
    public static double getFlywheelVelocity(double goalDist) {
        double rawVelocity = (0.000331549 * Math.pow(goalDist, 3))
                - (0.0542535 * Math.pow(goalDist, 2))
                + (8.92361 * goalDist)
                + 1212.0;
        return MathFunctions.clamp(rawVelocity, 0.0, 2200.0);
    }

    /**
     * Updates the goal distance based on robot's current position from RobotState
     */
    public void updateGoalDistance() {
        this.goalDist = RobotState.getInstance().getDistanceToGoal();
    }

    /**
     * Gets the calculated flywheel velocity based on current distance to goal
     */
    public double getCalculatedFlywheelVelocity() {
        updateGoalDistance();
        return getFlywheelVelocity(goalDist);
    }

    /**
     * Gets the calculated hood angle based on current distance to goal
     */
    public double getCalculatedHoodAngle() {
        updateGoalDistance();
        return getHoodAngle(goalDist);
    }

    public InternalMechanisms(HardwareMap hardwareMap) {

        intakeStage1 = hardwareMap.get(DcMotorEx.class, "int1");
        intakeStage2 = hardwareMap.get(DcMotorEx.class, "int2");
        shootR = hardwareMap.get(DcMotorEx.class, "shootR");
        shootL = hardwareMap.get(DcMotorEx.class, "shootL");
        gate = hardwareMap.get(Servo.class, "gate");
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");
        hoodR = hardwareMap.get(Servo.class, "hoodR");
        hoodL = hardwareMap.get(Servo.class, "hoodL");

        intakeStage1.setDirection(DcMotorEx.Direction.FORWARD);
        intakeStage2.setDirection(DcMotorEx.Direction.REVERSE);
        shootR.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeStage1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeStage2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(19.8, 0, 0, 12.5960);
        shootR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shootL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        rgbLight.setPosition(0);
        ledTimer.reset();

        IOState = RoboStates.IDLE;
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public boolean isShotsFired(){
        return shotsFired;
    }
    public double getLedColor() {
        return rgbLight.getPosition();
    }
    public double getStage1Current() {
        return stage1Current;
    }
    public double getStage2Current() {
        return stage2Current;
    }
    public double getStageOneCurrentLimit() {
        return STAGE_ONE_CURRENT_LIMIT;
    }
    public double getStageTwoCurrentLimit() {
        return STAGE_TWO_CURRENT_LIMIT;
    }
    public double getFlywheelVelocity() {
        return flywheelVelocity;
    }
    public void increaseFlywheelVelocity(double increment) {
        flywheelVelocity = Math.min(flywheelVelocity + increment, MAX_FLYWHEEL_VELOCITY);
    }
    public void decreaseFlywheelVelocity(double decrement) {
        flywheelVelocity = Math.max(flywheelVelocity - decrement, MIN_FLYWHEEL_VELOCITY);
    }
    public void setFlywheelVelocity(double velocity) {
        flywheelVelocity = MathFunctions.clamp(velocity, MIN_FLYWHEEL_VELOCITY, MAX_FLYWHEEL_VELOCITY);
    }
    public double getHoodPosition() {
        return hoodPosition;
    }
    public void increaseHoodPosition(double increment) {
        hoodPosition = Math.min(hoodPosition + increment, MAX_HOOD_POSITION);
    }
    public void decreaseHoodPosition(double decrement) {
        hoodPosition = Math.max(hoodPosition - decrement, MIN_HOOD_POSITION);
    }
    public void setHoodPosition(double position) {
        hoodPosition = MathFunctions.clamp(position, MIN_HOOD_POSITION, MAX_HOOD_POSITION);
    }
    public double getGoalDist() {
        return goalDist;
    }
    private void updateRainbowLED() {
        double time = ledTimer.seconds();
        double t = (time % cycleDuration) / cycleDuration;
        double minPos = 0.280;
        double maxPos = 0.722;
        double rainbowPosition = minPos + (t * (maxPos - minPos));
        rgbLight.setPosition(rainbowPosition);
    }
    public void setState(RoboStates newState) {
        this.IOState = newState;
        runtime.reset();
        ledTimer.reset();
        gateTimer.reset();
        gateOpened = false;
        stage1Stopped = false;
        stage2Stopped = false;
        intakeGraceTimer.reset(); // Reset grace timer when state changes
    }
    public void update() {
        currentVelocity = shootR.getVelocity();
        stage1Current = intakeStage1.getCurrent(CurrentUnit.AMPS);
        stage2Current = intakeStage2.getCurrent(CurrentUnit.AMPS);
        updateGoalDistance();
        switch (IOState) {
            case IDLE:
                intakeStage1.setPower(0);
                intakeStage2.setPower(0);

                // Continuously update flywheel velocity and hood angle based on distance
                double idleVelocity = getCalculatedFlywheelVelocity();
                double idleHoodAngle = getCalculatedHoodAngle();

                shootR.setVelocity(idleVelocity);
                shootL.setVelocity(idleVelocity);
                hoodR.setPosition(idleHoodAngle);
                hoodL.setPosition(idleHoodAngle);

                updateRainbowLED();
                gate.setPosition(0.2);
                break;

            case SHOOT:
                intakeStage2.setPower(1);
                intakeStage1.setPower(1);
                break;

            case FULL_IDLE:
                intakeStage1.setPower(0);
                intakeStage2.setPower(0);
                shootR.setVelocity(0);
                shootL.setVelocity(0);
                rgbLight.setPosition(0.280);
                gate.setPosition(0.2);
                break;

            case INTAKE:
                gate.setPosition(0.2);
                rgbLight.setPosition(0.280);

                // Keep flywheel and hood updated during intake too
                double intakeVelocity = getCalculatedFlywheelVelocity();
                double intakeHoodAngle = getCalculatedHoodAngle();

                shootR.setVelocity(intakeVelocity);
                shootL.setVelocity(intakeVelocity);
                hoodR.setPosition(intakeHoodAngle);
                hoodL.setPosition(intakeHoodAngle);

                // Only check current limits after grace period has passed
                if (intakeGraceTimer.seconds() > INTAKE_GRACE_PERIOD) {
                    if (stage1Current >= STAGE_ONE_CURRENT_LIMIT) {
                        stage1Stopped = true;
                    }
                    if (stage2Current >= STAGE_TWO_CURRENT_LIMIT) {
                        stage2Stopped = true;
                        rgbLight.setPosition(0.5);
                    }
                }

                // Set motor powers based on flags
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

            case AUTO_SCORE:
                // Get calculated values based on distance to goal
                double calculatedVelocity = getCalculatedFlywheelVelocity();
                double calculatedHoodAngle = getCalculatedHoodAngle();

                // Set flywheel velocity (TPS) - should already be near target from IDLE/INTAKE
                shootR.setVelocity(calculatedVelocity);
                shootL.setVelocity(calculatedVelocity);

                // Set hood angle (0 = most angle, 0.9 = lowest angle)
                hoodR.setPosition(calculatedHoodAngle);
                hoodL.setPosition(calculatedHoodAngle);

                // Calculate threshold as percentage of target velocity
                double velocityThreshold = calculatedVelocity * 0.95;

                // Flywheels should already be at speed, but check anyway with time fallback
                if (currentVelocity >= velocityThreshold || gateTimer.seconds() > 0.3) {
                    rgbLight.setPosition(0.5);
                    gate.setPosition(0.64);
                    if (!gateOpened) {
                        gateTimer.reset();
                        gateOpened = true;
                    }
                    if (gateOpened && gateTimer.seconds() > 0.18) {
                        intakeStage1.setPower(1);
                        intakeStage2.setPower(1);
                    }
                }
                break;
        }
    }
}
