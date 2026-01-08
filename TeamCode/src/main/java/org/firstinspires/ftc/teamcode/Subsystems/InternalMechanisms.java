package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private DcMotorEx intake = null;
    private DcMotorEx turretMotor;
    private DcMotorEx shootR = null;
    private DcMotorEx shootL = null;
    private Servo gate = null;
    private Servo rgbLight;
    private RoboStates IOState = RoboStates.IDLE;
    private double currentVelocity = 0;
    double cycleDuration = 0.7;
    private ElapsedTime ledTimer = new ElapsedTime();

    public InternalMechanisms(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotorEx.class, "int");
        shootR = hardwareMap.get(DcMotorEx.class, "shootR");
        shootL = hardwareMap.get(DcMotorEx.class, "shootL");
        gate = hardwareMap.get(Servo.class, "gate");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");

        intake.setDirection(DcMotorEx.Direction.FORWARD);
        shootR.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.FORWARD);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                intake.setPower(0);
                updateRainbowLED();
                gate.setPosition(0.6);
                shootR.setVelocity(0);
                shootL.setVelocity(0);
                //center turret
                //start LED to rainbow
                break;

            case INTAKE:
                gate.setPosition(0.6);
                intake.setPower(1);
                break;

            case SHOOT:
                gate.setPosition(0.2);
                intake.setPower(1);
                break;

            case FAR_SIDE:
                shootR.setVelocity(1500);
                shootL.setVelocity(1500);
                if (currentVelocity>=1500){
                    gate.setPosition(0.2);
                    intake.setPower(1);
                }
                break;

            case CLOSE_SIDE:
                shootR.setVelocity(1300);
                shootL.setVelocity(1300);
                if (currentVelocity>=1300){
                    gate.setPosition(0.2);
                    intake.setPower(1);
                }
                break;

            case AUTO_SCORE:
                shootR.setVelocity(1200);
                shootL.setVelocity(1200);
                if (currentVelocity>=1200){
                    gate.setPosition(0.2);
                    intake.setPower(1);
                }
                break;

            case AUTO_SPINUP:
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