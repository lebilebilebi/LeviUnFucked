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
        INTAKE,
        FIRST_BALL_STOP,
        FULL_STOP,
        SHOOT,
        FAR_SIDE,
        CLOSE_SIDE,
        LED_TEST_RAINBOW,
        LED_TEST_FLASHING,
        AUTO_SCORE,
        AUTO_SPINUP
    }

    // 2. Hardware variables
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
    private double goalDist = 0;
    private double goalX = 0;
    private double goalY = 0;
    private ElapsedTime ledTimer = new ElapsedTime();
    private static final double CURRENT_LIMIT_AMPS = 8.5;
    private static final double MAX_FLYWHEEL_VELOCITY = 2200;
    private static final double MIN_FLYWHEEL_VELOCITY = 0;
    private static final double MAX_HOOD_POSITION = 0.9;
    private static final double MIN_HOOD_POSITION = 0;
    public static double getHoodAngle(double goalDist) {
        double rawAngle = (0.00000120563 * Math.pow(goalDist, 3))
                - (0.000235615 * Math.pow(goalDist, 2))
                + (0.00615079 * goalDist)
                + 0.57;

        // Clamps the result to be safe for the servo
        return MathFunctions.clamp(rawAngle, 0.0, 0.9);
    }
    public static double getFlywheelVelocity(double goalDist) {
        double rawVelocity = (0.000331549 * Math.pow(goalDist, 3))
                - (0.0542535 * Math.pow(goalDist, 2))
                + (8.92361 * goalDist)
                + 1212.0;

        // Clamps the result to be safe for the motor
        return MathFunctions.clamp(rawVelocity, 0.0, 2200.0);
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
    }



    public double getVelocity() {
        return currentVelocity;
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
    public double getCurrentLimitAmps() {
        return CURRENT_LIMIT_AMPS;
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

    public void setGoalPose(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    public void updateGoalDist(double currentX, double currentY) {
        double deltaX = goalX - currentX;
        double deltaY = goalY - currentY;
        goalDist = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
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
    }
    public void update() {
        currentVelocity = shootR.getVelocity();
        stage1Current = intakeStage1.getCurrent(CurrentUnit.AMPS);
        stage2Current = intakeStage2.getCurrent(CurrentUnit.AMPS);

        // Apply flywheel and hood values
        shootR.setVelocity(flywheelVelocity);
        shootL.setVelocity(flywheelVelocity);
        hoodR.setPosition(hoodPosition);
        hoodL.setPosition(hoodPosition);

        switch (IOState) {
            case IDLE:
                intakeStage1.setPower(0);
                intakeStage2.setPower(0);

                updateRainbowLED();

                gate.setPosition(0.6);
                //center turret
                //start LED to rainbow
                break;

            case INTAKE:
                gate.setPosition(0.6);
                intakeStage1.setPower(1);
                intakeStage2.setPower(1);
                break;

            case FIRST_BALL_STOP:
                intakeStage2.setPower(0);
                gate.setPosition(0.6);
                break;

            case FULL_STOP:
                intakeStage1.setPower(0);
                gate.setPosition(0.6);
                break;

            case SHOOT:
                gate.setPosition(0.2);
                intakeStage1.setPower(1);
                intakeStage2.setPower(1);
                break;

            case FAR_SIDE:
                rgbLight.setPosition(0.280);
                shootR.setVelocity(2100);
                shootL.setVelocity(2100);
                if (currentVelocity>=2100){
                    rgbLight.setPosition(0.5);
                    gate.setPosition(0.2);
                    intakeStage1.setPower(1);
                    intakeStage2.setPower(1);
                }
                break;

            case CLOSE_SIDE:
                rgbLight.setPosition(0.280);
                shootR.setVelocity(1300);
                shootL.setVelocity(1300);
                if (currentVelocity>=1300){
                    rgbLight.setPosition(0.5);
                    gate.setPosition(0.2);
                    intakeStage1.setPower(1);
                    intakeStage2.setPower(1);}
                break;

            case AUTO_SCORE:
                shootR.setVelocity(1150);
                shootL.setVelocity(1150);
                if (currentVelocity>=1150){
                    rgbLight.setPosition(0.5);
                    gate.setPosition(0.2);
                    intakeStage1.setPower(1);
                    intakeStage2.setPower(1);}
                break;

            case AUTO_SPINUP:
                intakeStage1.setPower(1);
                intakeStage2.setPower(1);
                rgbLight.setPosition(0.280);
                gate.setPosition(0.6);
                shootR.setVelocity(1200);
                shootL.setVelocity(1200);
                break;

            case LED_TEST_RAINBOW:
                gate.setPosition(0.2);
                break;

            case LED_TEST_FLASHING:
                gate.setPosition(0.6);
                break;
        }
    }
}