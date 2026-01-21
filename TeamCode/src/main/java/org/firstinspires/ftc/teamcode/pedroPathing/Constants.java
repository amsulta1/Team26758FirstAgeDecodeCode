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

public class Constants {
    //TODO: Set mass of robot
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.80)
            .forwardZeroPowerAcceleration(-34.86982212235985)
            .lateralZeroPowerAcceleration(-48.0917597945857);

    //TODO: Set offset of odometry pods in POD(x/y) and hardware map names + encoder directions if needed
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.0030155604382137973)
            .strafeTicksToInches(0.0030187143237504835)
            .turnTicksToInches(-0.0029334360327087825)
            .leftPodY(6.25)
            .rightPodY(-6.25)
            .strafePodX(-6.4375)
            .leftEncoder_HardwareMapName("rightFront")
            .rightEncoder_HardwareMapName("leftFront")
            .strafeEncoder_HardwareMapName("rightBack")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
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
            .xVelocity(64.62606674453679)
            .yVelocity(52.04324042497011);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
