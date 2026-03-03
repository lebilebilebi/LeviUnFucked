package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.robot;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.SquIDController;

import lombok.Setter;

public class Drive extends WSubsystem{
    private SquIDController headingController = new SquIDController(0.0);
    private SquIDController translationalController = new SquIDController(0.0);
    private Vector2d errorVector = new Vector2d();
    private double headingError_RAD;
    @Setter
    private Point targetPoint = new Point();
    private final DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;
    private final robot robot;
    private double robotX_in, robotY_in, robotHeading_Rad, robotVelocityX_MPS, robotVelocityY_MPS;
    private double drive, strafe, turn;
    private double leftFrontPow, leftRearPow, rightFrontPow, rightRearPow;

    public Drive(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear,
                 GoBildaPinpointDriver pinpoint, Telemetry  telemetry, robot robot) {
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront = rightFront;
        this.rightRear = rightRear;

        // TODO: REVERSE WHEELS HERE

        this.pinpoint = pinpoint;

        // TODO: PUT OFFSETS AND ENCODER DIRECTIONS HERE

        this.telemetry = telemetry;
        this.robot = robot;
    }

    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }

    public void setLocation(Point point) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, point.getX(), point.getY(),
                AngleUnit.RADIANS, Math.toRadians(point.getHeading())));
    }

    public void driveToTarget(){
        headingError_RAD = AngleUnit.normalizeRadians(Math.toRadians(targetPoint.getHeading()) -
                robotHeading_Rad);
        errorVector = new Vector2d(targetPoint.getX() - robotX_in, targetPoint.getY() - robotY_in);


        //Rotate vector
        errorVector = errorVector.rotateBy(-Math.toDegrees(robotHeading_Rad));

        turn = - headingController.calculate(headingError_RAD);
        drive = translationalController.calculate(errorVector.getX());
        strafe = - translationalController.calculate(errorVector.getY());
    }

    public void stickInput(double drive, double strafe, double turn){
        final double MIN_POW = 0.1;
        this.drive = drive * (1.0 - MIN_POW) + Math.signum(drive * drive);
        this.strafe = strafe * (1.0 - MIN_POW) + Math.signum(strafe * strafe);
        this.turn = turn * (1.0 - MIN_POW) + Math.signum(turn * turn);

        Vector2d powerVec = new Vector2d(this.drive, this. strafe);
        powerVec = powerVec.rotateBy(Math.toDegrees(robotHeading_Rad) + 90); //add deg for where drivers are
        this.drive = powerVec.getX();
        this.strafe = powerVec.getY();
    }

    public void tuneHeading(double kSQ) {
        headingController.setkSQ(kSQ);
        telemetry.addData("Position", robotHeading_Rad);
        telemetry.addData("Target", Math.toRadians(targetPoint.getHeading()));
    }

    public void tuneTranslational(double kSQ) {
        telemetry.addData("Position", errorVector.magnitude());
        telemetry.addData("Target", 0.0);
    }

    @Override
    public void read() {
        pinpoint.update();
        robotX_in = pinpoint.getPosX(DistanceUnit.INCH);
        robotY_in = pinpoint.getPosY(DistanceUnit.INCH);
        robotHeading_Rad = pinpoint.getHeading(AngleUnit.RADIANS);
        robotVelocityX_MPS = pinpoint.getVelX(DistanceUnit.METER);
        robotVelocityY_MPS = pinpoint.getVelY(DistanceUnit.METER);

        //NOT NEEDED BUT MAYBE
        //robotHeading_Rad = AngleUnit.normalizeRadians(robotHeading_Rad);


    }

    @Override
    public void loop() {
        leftFrontPow = drive + strafe + turn;
        leftRearPow = drive - strafe + turn;
        rightFrontPow = drive - strafe - turn;
        rightRearPow = drive + strafe - turn;
    }

    @Override
    public void write() {
        leftFront.setPower(leftFrontPow);
        leftRear.setPower(leftRearPow);
        rightFront.setPower(rightFrontPow);
        rightRear.setPower(rightRearPow);
    }
}
