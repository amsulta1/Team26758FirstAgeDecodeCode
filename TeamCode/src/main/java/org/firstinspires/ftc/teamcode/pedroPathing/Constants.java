package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
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

public class Constants {
    public static PIDFCoefficients shooterConstants = new PIDFCoefficients(50, 0, 109.5, 15.1);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8)
            .forwardZeroPowerAcceleration(-34.98067785792871)
            .lateralZeroPowerAcceleration(-53.71272384623891)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.02, 0.02))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.05, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.6, 0, 0.1, 0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(5, 0, 0.18, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01))
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.00002);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .xVelocity(70.30400855931663)
            .yVelocity(58.936486330917965)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(-0.002961043207970234)
            .strafeTicksToInches(-0.0029860905682239695)
            .turnTicksToInches(-0.002929387801266088)
            .leftPodY(6.25)
            .rightPodY(-6.25)
            .strafePodX(-7)
            .leftEncoder_HardwareMapName("rightFront")
            .rightEncoder_HardwareMapName("leftFront")
            .strafeEncoder_HardwareMapName("rightBack")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
    //TODO: Set mass of robot
    /*public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.80)
            .forwardZeroPowerAcceleration(-34.86982212235985)
            .lateralZeroPowerAcceleration(-71.53268917544126)
            .translationalPIDFCoefficients(new PIDFCoefficients(-0.135f, 0, -0.03f, -0.02f))
            .headingPIDFCoefficients(new PIDFCoefficients(-1.4f, 0, -0.034f, -0.02f));
            //.translationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))
            //.headingPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0));

    //TODO: Set offset of odometry pods in POD(x/y) and hardware map names + encoder directions if needed
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.0030155604382137973)
            .strafeTicksToInches(0.0030187143237504835)
            .turnTicksToInches(-0.0029334360327087825)
            .leftPodY(-6.25)
            .rightPodY(6.25)
            .strafePodX(-6.4375)
            .rightEncoder_HardwareMapName("rightFront")
            .leftEncoder_HardwareMapName("leftFront")
            .strafeEncoder_HardwareMapName("rightBack")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65.28005951083324)
            .yVelocity(54.82756187062718);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }*/
}
