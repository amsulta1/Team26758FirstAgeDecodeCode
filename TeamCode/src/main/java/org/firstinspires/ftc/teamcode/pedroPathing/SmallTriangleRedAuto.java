package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "SmallTriangleRedAuto")
public class SmallTriangleRedAuto extends LinearOpMode {
    private Follower follower;
    private DcMotorEx shooterMotor;
    private Servo intakeServo2;
    private DcMotor intakeMotor;
    private Servo intakeServo;
    private Timer pathTimer, actionTimer, opModeTimer;
    private int pathState;
    float farShotVS = 1700f;
    private ElapsedTime servoTimer = new ElapsedTime();
    private final Pose startPose = new Pose(87, 8.25, Math.toRadians(270));
    private final Pose scorePose = new Pose(87.5, 20, Math.toRadians(245));
    private final Pose autoEndPose = new Pose(85.5f, 40, Math.toRadians(90));
    private final Pose getReadyForPickupFirstSet = new Pose(100, 36, Math.toRadians(0));
    private final Pose pickUpFirstSet = new Pose (133.25, 36f, Math.toRadians(0));

    private Path scorePreload;
    private PathChain getReadyToGrab, grabPathChain, scoreAgain, endingPathChain;


    private void ServoState(boolean IfFalseClosed){
        if(!IfFalseClosed){
            //closed
            intakeServo2.setPosition(0.12f);
            servoTimer.reset();
            while(servoTimer.milliseconds() < 150){}
            intakeServo.setPosition(0.59f);
            //closed
            //intake 2 servo 0.66
            //intake servo 0.82
        }else{
            //open
            boolean openingManuever =false;
            if(intakeServo2.getPosition()> 0.35f){
                openingManuever = true;
                intakeServo.setPosition(0.69f);
                servoTimer.reset();
                while (servoTimer.milliseconds() < 100){
                }
            }
            intakeServo2.setPosition(0.01f);
            if(openingManuever){
                while (servoTimer.milliseconds() < 550){
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
    private void setPathState(int newPathState){ pathState = newPathState; }
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                shooterMotor.setVelocityPIDFCoefficients(50.0f, 0.3549f, 96.1f, 10.0f);
                shooterMotor.setVelocity(farShotVS);
                follower.followPath(scorePreload);
                setPathState(2);
                break;
            case 2:
                if(!follower.isBusy()){
                    sendToShooting(3);
                    sleep(1300);
                    sendToShooting(2);
                    sleep(1300);
                    sendToShooting(1);
                    sleep(1000);
                    shooterMotor.setVelocityPIDFCoefficients(50.0f, 0.3549f, 96.1f, 10.0f);
                    shooterMotor.setVelocity(0);
                    setPathState(3);
                    follower.followPath(getReadyToGrab);
                    ServoState(true);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0.5f);
                    follower.followPath(grabPathChain);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    ServoState(false);
                    intakeMotor.setPower(0);
                    follower.followPath(scoreAgain);
                    shooterMotor.setVelocityPIDFCoefficients(50.0f, 0.3549f, 96.1f, 10.0f);
                    shooterMotor.setVelocity(farShotVS);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    sendToShooting(3);
                    sleep(1300);
                    sendToShooting(2);
                    sleep(1300);
                    sendToShooting(1);
                    sleep(1000);
                    setPathState(6);
                    ServoState(true);
                    follower.followPath(endingPathChain);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    setPathState(7);
                    shooterMotor.setVelocityPIDFCoefficients(50.0f, 0.3549f, 96.1f, 10.0f);
                    shooterMotor.setVelocity(0);
                    intakeMotor.setPower(0);
                    requestOpModeStop();
                }
                break;
        }
    }
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        getReadyToGrab = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, getReadyForPickupFirstSet))
                .setLinearHeadingInterpolation(scorePose.getHeading(), getReadyForPickupFirstSet.getHeading())
                .build();
        grabPathChain = follower.pathBuilder()
                .addPath(new BezierLine(getReadyForPickupFirstSet, pickUpFirstSet))
                .setLinearHeadingInterpolation(getReadyForPickupFirstSet.getHeading(), pickUpFirstSet.getHeading())
                .build();
        scoreAgain = follower.pathBuilder()
                .addPath(new BezierLine(pickUpFirstSet, scorePose))
                .setLinearHeadingInterpolation(pickUpFirstSet.getHeading(), scorePose.getHeading())
                .build();
        endingPathChain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, autoEndPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), autoEndPose.getHeading())
                .build();


    }

    @Override
    public void runOpMode() {
        //get shooterMotor
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeSystem");
        intakeServo = hardwareMap.get(Servo.class, "PBTSS");
        intakeServo2 = hardwareMap.get(Servo.class, "PBTSS2");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ServoState(false);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            setPathState(0);
            opModeTimer = new Timer();
            opModeTimer.resetTimer();
            while (opModeIsActive()) {
                // OpMode loop
                follower.update();
                autonomousPathUpdate();
                // Feedback to Driver Hub for debugging
                telemetry.addData("path state", pathState);
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();
            }
        }
    }
}
