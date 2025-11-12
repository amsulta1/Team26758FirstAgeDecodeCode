package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "SmallTriangleBlueAuto")
public class SmallTriangleBlueAuto extends LinearOpMode {
    private Follower follower;
    private DcMotor shooterMotor;
    private float sendToShotSystem = 0.2991f;
    private float holdEverythingIn = 0.5495f;
    private float relaxed = 0.8f;
    private DcMotor intakeMotor;
    boolean intheMiddleOfShooting = false;
    private Servo intakeServo;
    private Timer pathTimer, actionTimer, opModeTimer;
    private int pathState;
    float timerNum = 0;
    boolean shootingBallsCurrently = false;
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(0, 0, Math.toRadians(0));
    private final Pose autoEndPose = new Pose(39, 33, Math.toRadians(90));
    private final Pose getReadyForPickupFirstSet = new Pose(56, 36, Math.toRadians(180));
    private final Pose pickUpFirstSet = new Pose (19, 36, Math.toRadians(180));

    private Path scorePreload;
    private PathChain getReady1PathChain, grab1PathChain, score1PathChain, finalDestination;

    public void shootAll3Balls() {
        shootingBallsCurrently = true;
        shooterMotor.setPower(0.85f);
        if(!intheMiddleOfShooting) {
            timerNum = (float) opModeTimer.getElapsedTimeSeconds();
            intheMiddleOfShooting = true;
        }
        if(opModeTimer.getElapsedTimeSeconds() >= (2 + timerNum)){
            intakeServo.setPosition(sendToShotSystem);
            if(opModeTimer.getElapsedTime() >= (5+timerNum)){
                shooterMotor.setPower(0);
                shootingBallsCurrently = false;
                intheMiddleOfShooting = false;
            }
        }
    }
    private void setPathState(int newPathState){ pathState = newPathState; }
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    shootAll3Balls();
                    if(!shootingBallsCurrently) {
                        follower.followPath(getReady1PathChain);
                        intakeServo.setPosition(relaxed);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(grab1PathChain);
                    intakeMotor.setPower(0.5f);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    intakeServo.setPosition(holdEverythingIn);
                    intakeMotor.setPower(0);
                    follower.followPath(score1PathChain);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    shootAll3Balls();
                    if(!shootingBallsCurrently) {
                        intakeServo.setPosition(relaxed);
                        intakeMotor.setPower(0);
                        shooterMotor.setPower(0);
                        setPathState(5);
                        follower.followPath(finalDestination);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    setPathState(6);
                    stop();
                }
                break;
        }
    }
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        getReady1PathChain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, getReadyForPickupFirstSet))
                .setLinearHeadingInterpolation(scorePose.getHeading(), getReadyForPickupFirstSet.getHeading())
                .build();
        grab1PathChain = follower.pathBuilder()
                .addPath(new BezierLine(getReadyForPickupFirstSet, pickUpFirstSet))
                .setLinearHeadingInterpolation(getReadyForPickupFirstSet.getHeading(), pickUpFirstSet.getHeading())
                .build();
        score1PathChain = follower.pathBuilder()
                .addPath(new BezierLine(pickUpFirstSet, scorePose))
                .setLinearHeadingInterpolation(pickUpFirstSet.getHeading(), scorePose.getHeading())
                .build();
        finalDestination = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, autoEndPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), autoEndPose.getHeading())
                .build();

    }

    @Override
    public void runOpMode() {
        //get shooterMotor
        shooterMotor = hardwareMap.get(DcMotor.class, "Shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeSystem");
        intakeServo = hardwareMap.get(Servo.class, "PBTSS");
        intakeServo.setPosition(holdEverythingIn);
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
