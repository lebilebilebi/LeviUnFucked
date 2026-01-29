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
            .mass(11.88412);
            //.forwardZeroPowerAcceleration(-33)
            //.lateralZeroPowerAcceleration(-54)
            //.centripetalScaling(0.0005)
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0.0085,0.022))
            //.headingPIDFCoefficients(new PIDFCoefficients(0.42, 0, 0.0031, 0.032))
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006, 0.0, 0.00005, 0.6, 0.035));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(80.42785)
            //.yVelocity(52.5)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .strafePodX(5.9133)
            .forwardPodY(4.9370)
            .forwardTicksToInches(0.002005714155109081)
            .strafeTicksToInches(0.0019878380712762063)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
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