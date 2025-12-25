package org.firstinspires.ftc.teamcode.auto.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.5)
            .forwardZeroPowerAcceleration(-36.12)
            .lateralZeroPowerAcceleration(-48.11)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0.0085,0.022))
            .headingPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.0031, 0.032))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006, 0.0, 0.00005, 0.6, 0.035));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8)
            .xVelocity(73.99)
            .yVelocity(65.35)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .strafePodX(3.75)
            .forwardPodY(2.5)
            .forwardTicksToInches(0.002965778801287011)
            .strafeTicksToInches(0.002940854923108392)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardEncoder_HardwareMapName("fr")
            .strafeEncoder_HardwareMapName("bl")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}