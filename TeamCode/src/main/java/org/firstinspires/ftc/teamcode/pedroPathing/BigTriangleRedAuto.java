package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "BigTriangleRedAuto")
public class BigTriangleRedAuto extends LinearOpMode {
    private Follower follower;
    private int pathState = 0;

    private DcMotorEx shooterMotor;
    private DcMotor intakeMotor;
    private Servo intakeServo;
    private Servo intakeServo2;
    private float closeRangeVS = 1110f;
    private final float intakeMotorSpeed = 0.5f;

    private final Pose startPose = new Pose(110.25f, 135.25f, Math.toRadians(180));
    private final Pose getReadyToGrab = new Pose(100, 84, Math.toRadians(0));
    private final Pose grab = new Pose(127, 84, Math.toRadians(0));
    private final Pose scorePose = new Pose (87.5f, 135.25f, Math.toRadians(180));
    private final Pose EndingPose = new Pose(100, 80, Math.toRadians(90));

    private PathChain goToScoreOg, getReadyToGrabStuff, grabStuff, scoreSecondSet, Ending;


    void buildPaths(){
        goToScoreOg = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        getReadyToGrabStuff = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, getReadyToGrab))
                .setLinearHeadingInterpolation(scorePose.getHeading(), getReadyToGrab.getHeading())
                .build();
        grabStuff = follower.pathBuilder()
                .addPath(new BezierLine(getReadyToGrab, grab))
                .setLinearHeadingInterpolation(getReadyToGrab.getHeading(), grab.getHeading())
                .build();
        scoreSecondSet = follower.pathBuilder()
                .addPath(new BezierLine(grab, scorePose))
                .setLinearHeadingInterpolation(grab.getHeading(), scorePose.getHeading())
                .build();
        Ending = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, EndingPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), EndingPose.getHeading())
                .build();
    }
    void setPathState(int setPathStateTo){
        pathState = setPathStateTo;
    }


    private void ServoState(boolean IfFalseClosed) {
        ElapsedTime servoTimer;
        servoTimer = new ElapsedTime();
        if (!IfFalseClosed) {
            //closed
            intakeServo2.setPosition(0.12f);
            servoTimer.reset();
            while (servoTimer.milliseconds() < 150) {
            }
            intakeServo.setPosition(0.59f);
        } else {
            //open
            boolean openingManuever = false;
            if (intakeServo2.getPosition() > 0.35f) {
                openingManuever = true;
                intakeServo.setPosition(0.69f);
                servoTimer.reset();
                while (servoTimer.milliseconds() < 100) {
                }
            }
            intakeServo2.setPosition(0.01f);
            if (openingManuever) {
                while (servoTimer.milliseconds() < 550) {
                }
            }
            intakeServo.setPosition(0.75f);
        }
    }
    void sendToShooting(int howManyBalls){
        ElapsedTime servoTimer = new ElapsedTime();
        if(howManyBalls == 2){
            //shooting with 2 balls
            intakeServo2.setPosition(0.39f);
        }else if(howManyBalls == 1){
            //shooting the last ball
            intakeServo2.setPosition(0.59f);
        }else{
            intakeServo2.setPosition(0.2f);
            servoTimer.reset();
            while (servoTimer.milliseconds() < 250){
            }
            intakeServo.setPosition(0.69f);
        }

    }

    void autonomousPathManagement(){
        switch (pathState){
            case 0:
                shooterMotor.setVelocityPIDFCoefficients(50.0f, 0.3549f, 96.1f, 10.0f);
                shooterMotor.setVelocity(closeRangeVS);
                follower.followPath(goToScoreOg);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    sendToShooting(3);
                    sleep(1300);
                    sendToShooting(2);
                    sleep(1300);
                    sendToShooting(1);
                    sleep(1000);
                    shooterMotor.setVelocityPIDFCoefficients(50.0f, 0.3549f, 96.1f, 10.0f);
                    shooterMotor.setVelocity(0);
                    ServoState(true);
                    follower.followPath(getReadyToGrabStuff);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    intakeMotor.setPower(intakeMotorSpeed);
                    follower.followPath(grabStuff);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    ServoState(false);
                    intakeMotor.setPower(0);
                    shooterMotor.setVelocityPIDFCoefficients(50.0f, 0.3549f, 96.1f, 10.0f);
                    shooterMotor.setVelocity(closeRangeVS);
                    follower.followPath(scoreSecondSet);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    sendToShooting(3);
                    sleep(1300);
                    sendToShooting(2);
                    sleep(1300);
                    sendToShooting(1);
                    sleep(1000);
                    shooterMotor.setVelocityPIDFCoefficients(50.0f, 0.3549f, 96.1f, 10.0f);
                    shooterMotor.setVelocity(0);
                    intakeMotor.setPower(0);
                    ServoState(true);
                    follower.followPath(Ending);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    requestOpModeStop();
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeSystem");
        intakeServo = hardwareMap.get(Servo.class, "PBTSS");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo2 = hardwareMap.get(Servo.class, "PBTSS2");
        follower.setStartingPose(startPose);
        ServoState(true);
        buildPaths();
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            setPathState(0);
            while (opModeIsActive()) {
                // OpMode loop
                follower.update();
                autonomousPathManagement();
            }
        }
    }
}
