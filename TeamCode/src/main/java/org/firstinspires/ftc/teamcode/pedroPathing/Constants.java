package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Stopped at Drive tuning due to space limitations
// Check the ticksToInch (First steps) - robot seems to be measuring wrong

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9)
            .forwardZeroPowerAcceleration(-38.82019704346193)
            .lateralZeroPowerAcceleration(-75.61359656171102)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.35, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0.1, 0.1, 0.01));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.0029258769839086633)
            .strafeTicksToInches(.002852051308326529)
            .turnTicksToInches(.00291887512018613)
            .leftPodY(5.34375)
            .rightPodY(-5.34375)
            .strafePodX(-7.5625)
            .leftEncoder_HardwareMapName("bl")
            .rightEncoder_HardwareMapName("fr")
            .strafeEncoder_HardwareMapName("br")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .threeWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(76.33621307311817)
            .yVelocity(55.02929246176179)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
}
