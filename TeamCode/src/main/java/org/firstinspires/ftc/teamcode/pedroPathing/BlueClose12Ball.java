package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "BlueClose12Ball")
public class BlueClose12Ball extends LinearOpMode {
    private Follower follower;
    private int pathState = 0;
    private float farShotVS = 1100;
    private float VELOCITY = 0;
    HuskyLens camera;
    private float intakePower = 0.5f;
    ElapsedTime sweeperTime;
    double kP = 0.002;
    double error = 0;
    double lastError = 0;
    double goalX = -400;  // offset
    double angleTolerance = 0.1;
    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;
    private Servo intakeServo;
    private Servo intakeServo2;
    private DcMotor intakeMotor;
    private DcMotorEx shooterMotor;
    private final Pose startPose = new Pose(26, 129.5, Math.toRadians(323));
    private final Pose scorePose3 = new Pose(53, 97, Math.toRadians(315));

    private final Pose scorePose = new Pose(48, 103, Math.toRadians(305));
    private final Pose scorePosePreload = new Pose(48, 103, Math.toRadians(320));

    private final Pose ReadyUp3 = new Pose(54, 35.5, Math.toRadians(180));
    private final Pose Grab3 = new Pose(10, 35.5, Math.toRadians(180));
    private final Pose ReadyUp2 = new Pose(54, 60, Math.toRadians(180));
    private final Pose Grab2 = new Pose(14, 60, Math.toRadians(180));
    //private final Pose GateOpen = new Pose(128, 72, Math.toRadians(90));
    private final Pose ReadyUp1 = new Pose(54, 84, Math.toRadians(180));
    private final Pose Grab1 = new Pose(16, 84, Math.toRadians(180));
    private final Pose autoEnd = new Pose(59, 55, Math.toRadians(90));

    private Path scorePreload;
    private PathChain R1Chain, G1Chain, score1Chain, R2Chain, G2Chain, score2Chain, R3Chain, G3Chain, endChain;

    private void buidPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePosePreload));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePosePreload.getHeading());

        R1Chain = follower.pathBuilder()
                .addPath(new BezierLine(scorePosePreload, ReadyUp1))
                .setLinearHeadingInterpolation(scorePosePreload.getHeading(), ReadyUp1.getHeading())
                .build();
        G1Chain = follower.pathBuilder()
                .addPath(new BezierLine(ReadyUp1, Grab1))
                .setLinearHeadingInterpolation(ReadyUp1.getHeading(), Grab1.getHeading())
                .build();
        score1Chain = follower.pathBuilder()
                .addPath(new BezierLine(Grab1, scorePose))
                .setLinearHeadingInterpolation(Grab1.getHeading(), scorePose.getHeading())
                .build();
        R2Chain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, ReadyUp2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), ReadyUp2.getHeading())
                .build();
        G2Chain = follower.pathBuilder()
                .addPath(new BezierLine(ReadyUp2, Grab2))
                .setLinearHeadingInterpolation(ReadyUp2.getHeading(), Grab2.getHeading())
                .build();
        score2Chain = follower.pathBuilder()
                .addPath(new BezierCurve(Grab2, new Pose(46, 65), scorePose3))
                .setLinearHeadingInterpolation(Grab2.getHeading(), scorePose3.getHeading())
                .build();
        R3Chain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose3, ReadyUp3))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), ReadyUp2.getHeading())
                .build();
        G3Chain = follower.pathBuilder()
                .addPath(new BezierLine(ReadyUp3, Grab3))
                .setLinearHeadingInterpolation(ReadyUp3.getHeading(), Grab3.getHeading())
                .build();
        /*score3Chain = follower.pathBuilder()
                .addPath(new BezierLine(Grab3, scorePose))
                .setLinearHeadingInterpolation(Grab3.getHeading(), scorePose.getHeading())
                .build();*/
        endChain = follower.pathBuilder()
                .addPath(new BezierLine(Grab3, autoEnd))
                .setLinearHeadingInterpolation(Grab3.getHeading(), autoEnd.getHeading())
                .build();
    }
    void setPathState(int theNewPathState){pathState = theNewPathState;}
    private void ServoState(boolean IfFalseClosed){
        ElapsedTime servoTimer = new ElapsedTime();
        if(!IfFalseClosed){
            intakeServo2.setPosition(0.12f);
            servoTimer.reset();
            while(servoTimer.milliseconds() < 150){}
            intakeServo.setPosition(0.59f);
        }else{
            intakeServo2.setPosition(0.01f);
            intakeServo.setPosition(0.75f);
        }
    }
    void shotMotorManagement(){
        //JustForSlowingDown
        shooterMotor.setVelocityPIDFCoefficients(50.0, 0, 109.5, 15.1);
        if(VELOCITY == 0){
            if(shooterMotor.getVelocity() == 0){}
            else{
                if(shooterMotor.getVelocity() > 200) {
                    shooterMotor.setVelocity(-shooterMotor.getVelocity());
                }else{
                    shooterMotor.setVelocity(0);
                }
            }
        }
    }
    void sweeper(){
        sweeperTime.reset();
        shooterMotor.setVelocityPIDFCoefficients(50.0, 0, 109.5, 15.1);
        VELOCITY = -1050;
        shooterMotor.setVelocity(VELOCITY);

    }
    void scoreFunction(){
        shooterMotor.setVelocityPIDFCoefficients(50.0, 0, 109.5, 15.1);
        VELOCITY = farShotVS;
        shooterMotor.setVelocity(VELOCITY);
        shotMotorManagement();
        sleep(900);
        sendToShotRamp(3);
        sleep(700);
        sendToShotRamp(2);
        sleep(500);
        sendToShotRamp(1);
        sleep(1050);
        VELOCITY = 0;

    }
    void fixPositioning() {
        double goalErrorFirstTime = 0;
        //AprilTagDetection id20 = camera.blocks(20)[0];  // change null to get tag from id 20 from HuskyLens "camera"
        HuskyLens.Block[] myHuskyLensBlocks = camera.blocks();
        int id20 = 0;
        for (HuskyLens.Block myHuskyLensBlock_item : myHuskyLensBlocks) {
            id20 = myHuskyLensBlock_item.id;
            goalErrorFirstTime = myHuskyLensBlock_item.x;

        }

        //9 is red, 8 is blue
        // auto align logic

        float yaw = 0;
        if (id20 != 0) {
            error = 600 - goalErrorFirstTime + goalX;  // subtract the "tx from a limelight" received from HuskyLens from goalX

            if (Math.abs(error) < angleTolerance) {
                yaw = 0;
            } else {
                double pTerm = error * kP;

                curTime = getRuntime();
                double dT = curTime - lastTime;
                double dTerm = ((error - lastError) / dT) * kD;

                yaw = (float) Range.clip(pTerm + dTerm, -0.4, 0.4);

                lastError = error;
                lastTime = curTime;
            }
        } else {
            lastTime = getRuntime();
            lastError = 0;
        }
        follower.setTeleOpDrive(0, 0, yaw, true);
    }
    void sendToShotRamp(int howManyBalls){
        ElapsedTime servoTimer = new ElapsedTime();
        servoTimer.reset();
        if(howManyBalls == 2){
            //shooting with 2 balls
            intakeServo2.setPosition(0.39f);
        }else if(howManyBalls == 1){
            //shooting the last ball
            intakeServo2.setPosition(0.52f);
            //servoTimer.reset();
            //while (servoTimer.milliseconds() < 100){}
            intakeServo.setPosition(0.15f);

        }else{
            intakeServo2.setPosition(0.2f);
        }

    }
    void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    ElapsedTime alignmentTimer = new ElapsedTime();
                    follower.breakFollowing();
                    follower.startTeleopDrive();
                    while (alignmentTimer.milliseconds() < 200){
                        fixPositioning();
                    }
                    lastError = 0;
                    lastTime = getRuntime();
                    scoreFunction();
                    ServoState(true);
                    //score
                    follower.followPath(R1Chain);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    intakeMotor.setPower(intakePower);
                    follower.followPath(G1Chain);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    ServoState(false);
                    sweeper();
                    sleep(500);
                    follower.followPath(score1Chain);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    ElapsedTime alignmentTimer = new ElapsedTime();
                    follower.breakFollowing();
                    follower.startTeleopDrive();
                    while (alignmentTimer.milliseconds() < 200){
                        fixPositioning();
                    }
                    lastError = 0;
                    lastTime = getRuntime();
                    scoreFunction();
                    //score
                    ServoState(true);
                    follower.followPath(R2Chain);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    //stopShotMotor
                    //intake on
                    intakeMotor.setPower(intakePower);
                    //open up servo
                    follower.followPath(G2Chain);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    //score
                    intakeMotor.setPower(0);
                    sweeper();
                    ServoState(false);
                    follower.followPath(score2Chain);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    ElapsedTime alignmentTimer = new ElapsedTime();
                    follower.breakFollowing();
                    follower.startTeleopDrive();
                    while (alignmentTimer.milliseconds() < 200){
                        fixPositioning();
                    }
                    lastError = 0;
                    lastTime = getRuntime();
                    scoreFunction();
                    //score
                    ServoState(true);
                    follower.followPath(R3Chain);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    //stopShotMotor
                    //intake on
                    intakeMotor.setPower(intakePower);
                    //open up servo
                    follower.followPath(G3Chain);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    //score
                    intakeMotor.setPower(0);
                    sweeper();
                    ServoState(false);
                    follower.followPath(endChain);
                    setPathState(11);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    ElapsedTime alignmentTimer = new ElapsedTime();
                    follower.breakFollowing();
                    follower.startTeleopDrive();
                    while (alignmentTimer.milliseconds() < 200){
                        fixPositioning();
                    }
                    lastError = 0;
                    lastTime = getRuntime();
                    scoreFunction();
                    VELOCITY = 0;
                    intakeMotor.setPower(0);
                    ServoState(true);
                    follower.followPath(endChain);
                    setPathState(11);
                }
                break;
            default:
                break;
        }
    }
    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        camera = hardwareMap.get(HuskyLens.class, "camera");
        camera.initialize();
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeSystem");
        intakeServo = hardwareMap.get(Servo.class, "PBTSS");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        intakeServo2 = hardwareMap.get(Servo.class, "PBTSS2");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        buidPaths();
        setPathState(0);
        ServoState(false);
        sweeperTime = new ElapsedTime();
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                if(sweeperTime.milliseconds() > 700){VELOCITY = 0;}
                autonomousPathUpdate();
                shotMotorManagement();
                follower.update();
                telemetry.addData("Current Position: ", follower.getPose());
                telemetry.update();
            }
        }
    }
}
