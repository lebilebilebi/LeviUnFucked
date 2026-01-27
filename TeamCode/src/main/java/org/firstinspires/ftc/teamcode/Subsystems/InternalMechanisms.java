package org.firstinspires.ftc.teamcode.Subsystems;
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
    private DcMotorEx intakeStage1 = null;
    private DcMotorEx intakeStage2 = null;

    private DcMotorEx turretMotor;
    private DcMotorEx shootR = null;
    private DcMotorEx shootL = null;
    private Servo gate = null;
    private Servo rgbLight;
    private RoboStates IOState = RoboStates.IDLE;
    private double currentVelocity = 0;
    double stage1Current = intakeStage1.getCurrent(CurrentUnit.AMPS);
    double stage2Current = intakeStage2.getCurrent(CurrentUnit.AMPS);
    double cycleDuration = 0.7;
    private ElapsedTime ledTimer = new ElapsedTime();
    private static final double CURRENT_LIMIT_AMPS = 4.5;

    public InternalMechanisms(HardwareMap hardwareMap) {

        intakeStage1 = hardwareMap.get(DcMotorEx.class, "int1");
        intakeStage2 = hardwareMap.get(DcMotorEx.class, "int2");
        shootR = hardwareMap.get(DcMotorEx.class, "shootR");
        shootL = hardwareMap.get(DcMotorEx.class, "shootL");
        gate = hardwareMap.get(Servo.class, "gate");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");

        intakeStage1.setDirection(DcMotorEx.Direction.REVERSE);
        intakeStage2.setDirection(DcMotorEx.Direction.FORWARD);
        shootR.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.FORWARD);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeStage1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeStage2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(16, 0, 0, 16);
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

        switch (IOState) {
            case IDLE:
                intakeStage1.setPower(0);
                intakeStage2.setPower(0);

                updateRainbowLED();

                gate.setPosition(0.6);

                shootR.setVelocity(0);
                shootL.setVelocity(0);
                //center turret
                //start LED to rainbow
                break;

            case INTAKE:
                gate.setPosition(0.6);
                intakeStage1.setPower(1);
                intakeStage2.setPower(1);
                if (stage1Current > CURRENT_LIMIT_AMPS) {
                    intakeStage1.setPower(0.0);

                } else if (stage2Current > CURRENT_LIMIT_AMPS) {
                    intakeStage2.setPower(0.0);
                }
                break;

            case SHOOT:
                gate.setPosition(0.2);
                intakeStage1.setPower(1);
                intakeStage2.setPower(1);
                break;

            case FAR_SIDE:
                rgbLight.setPosition(0.280);
                shootR.setVelocity(1475);
                shootL.setVelocity(1475);
                if (currentVelocity>=1475){
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