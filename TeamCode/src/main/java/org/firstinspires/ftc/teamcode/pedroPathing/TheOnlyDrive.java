package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "My Leige, How Are You")
public class TheOnlyDrive extends LinearOpMode {
    private DcMotor rightFront;
    //take in 0.82  hold .67    shoot 0.3
    private DcMotor rightBack;
    private HuskyLens camera;
    private Pose farShotPose = new Pose(88, 18, Math.toRadians(245));
    private Pose closeShotPose = new Pose(72, 133.5, Math.toRadians(180));
    private Pose ParkingPose = new Pose(40, 35, Math.toRadians(90));
    private boolean shotMotorOn = false;
    private Follower follower;
    boolean usingAutoPosition = false;
    private DcMotorEx shooterMotor;
    private DcMotor leftFront;
    private float shotVelocity = 2000;
    float closeShot = 1140;
    float farShot = 1245;
    private Servo intakeServo;
    private Servo intakeServo2;
    /// Camera Stuff
    double kP = 0.002; //og 0.002
    double error = 0;
    double lastError = 0;
    double goalX = -400;  // offset
    double angleTolerance = 0.1;
    double kD = 0.0001; // og 0.0001
    double curTime = 0;
    double lastTime = 0;
    ///
    private DcMotor leftBack;
    float intakePower = 0.8f;
    private ElapsedTime servoTimer = new ElapsedTime();
    enum IntakeServoPos{
        Standby,
        Holding,
        SendToShooting1, //.22
        SendToShooting2, //.46
        SendToShooting3
    }
    IntakeServoPos currentServoState = IntakeServoPos.Standby;
    private DcMotor intakeMotor;
    TelemetryManager telemetryM;
    private final Pose autoEndPose = new Pose(85, 55, Math.toRadians(90));

    Pose weAreGoingTo = autoEndPose;
    boolean scoringInBlueGoal = true;
    //leftyimu
    //middleximu
    //rightyimu
    boolean intakeSpinning = false;
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        //1 ball .2491
        //2 ball .4493
        //3 ball .5495
        //send to shot .2991
        double max = 0;
        runtime = new ElapsedTime();
        initializationLogic();
        shooterMotor.setVelocityPIDFCoefficients(10.0f, 0.3549f, 96.1f, 10.0f);
        FollowerInit();
        camera.initialize();
        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            // Pre-run
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                // OpMode loop
                follower.update();
                telemetryM.update();
                movementLogic();
                IntakeMotorManagement();
                ServoManagement();
                shotMotorManagement();
                //make a shot, servo sending balls to shot system
                telemetryData();
            }
        }
    }
    public void telemetryData(){
        telemetry.addData("Intake Motor Position: ", intakeServo.getPosition());
        telemetry.addLine("0 = Intake Servo, 1 = Intake Motor, 2 = Shot Motor, 3 = Intake Servo 2");
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Scoring In Blue Goal: ", scoringInBlueGoal);
        telemetry.addData("Shot Motor VS: ", shooterMotor.getVelocity());
        telemetry.update();
    }
    private void FollowerInit(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(autoEndPose);
        follower.startTeleopDrive();
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }
    private void ServoState(boolean IfFalseClosed){
        if(!IfFalseClosed){
            //closed
            intakeServo2.setPosition(0.12f);
            servoTimer.reset();
            while(servoTimer.milliseconds() < 150){movementLogic();}
            intakeServo.setPosition(0.59f);
            //closed
            //intake 2 servo 0.66
            //intake servo 0.82
        }else{
            //open

            intakeServo2.setPosition(0.01f);
            intakeServo.setPosition(0.75f);
            //intake 2 servo 0.48
            //intake servo 1
        }
    }
    void sendToShotRamp(int howManyBalls){
        if(howManyBalls == 2){
            intakeServo2.setPosition(0.39f);
        }else if(howManyBalls == 1){
            intakeServo2.setPosition(0.52f);
            intakeServo.setPosition(0.16f);

        }else{
            intakeServo2.setPosition(0.2f);
        }

    }
    private void ServoManagement(){

        //holdin main: .91 second .13     second first, main afterward
        //shooting 3  main .82, it should be holding first though.
        //shooting 2  main 0.6 it should be from shooting 3.
        //shooting 1 main .52 secondary .62 from shooting 2 both of them together.
        if(gamepad2.a){
            servoTimer.reset();
            if(currentServoState == IntakeServoPos.SendToShooting3){
                //shooting with 2 balls
                currentServoState = IntakeServoPos.SendToShooting2;
                intakeServo2.setPosition(0.39f);
            }else if(currentServoState == IntakeServoPos.SendToShooting2){
                //shooting the last ball
                currentServoState = IntakeServoPos.SendToShooting1;
                intakeServo2.setPosition(0.52f);
                /*servoTimer.reset();
                while (servoTimer.milliseconds() < 100){
                    movementLogic();
                    shotMotorManagement();
                    IntakeMotorManagement();
                }*/
                intakeServo.setPosition(0.16f);

            }else{
                currentServoState = IntakeServoPos.SendToShooting3;
                intakeServo2.setPosition(0.2f);
            }
            /*if(currentServoState == IntakeServoPos.SendToShooting1){
                currentServoState = IntakeServoPos.SendToShooting2;
                intakeServo2.setPosition(0.60f);
                servoTimer.reset();
                while(servoTimer.milliseconds() < 300){
                    movementLogic();
                }
                intakeServo.setPosition(0.7f);
                //there are 2 balls
            }else if(currentServoState == IntakeServoPos.SendToShooting2){
                currentServoState = IntakeServoPos.SendToShooting1;
                //there is oneball
                servoTimer.reset();
                intakeServo2.setPosition(0.60f);
                while(servoTimer.milliseconds() < 300){
                    movementLogic();
                }
                servoTimer.reset();
                intakeServo.setPosition(0.67f);
                while(servoTimer.milliseconds() < 300){
                    movementLogic();
                }
                intakeServo2.setPosition(1f);

            } else{
                servoTimer.reset();
                currentServoState = IntakeServoPos.SendToShooting2;
                intakeServo2.setPosition(0.60f);
                while(servoTimer.milliseconds() < 300){
                    movementLogic();
                }
                intakeServo.setPosition(0.7f);
            }*/
            servoTimer.reset();
            sleep(150);
        }

        if(gamepad2.b){
            if(currentServoState == IntakeServoPos.Holding){
                currentServoState = IntakeServoPos.Standby;
                ServoState(true);
                gamepad2.rumble(500);
                //take in 1
            }else if(currentServoState == IntakeServoPos.Standby){
                currentServoState = IntakeServoPos.Holding;
                ServoState(false);
            }else {
                currentServoState = IntakeServoPos.Standby;
                ServoState(true);
                gamepad2.rumble(500);
            }
            sleep(150);
        }
    }
    private void shotMotorManagement(){
        if(gamepad2.left_trigger > 0){
            ServoState(true);
            currentServoState = IntakeServoPos.Standby;
            shotMotorOn = false;
        }
        if(gamepad2.y){
            shotVelocity = -500;
            shotMotorOn = true;
        }else{
            shotMotorOn = false;
        }
        if(gamepad2.left_bumper){
            gamepad2.setLedColor(0, 0, 265, 3000);
            intakeSpinning = false;
            shotVelocity = closeShot;
            intakeMotor.setPower(0);
            shotMotorOn = true;
            /*if(whichMotorIsDPad == 2) {
                gamepad2.rumble(1000);
                shooterMotor.setPower(DPadNumber);
            }else{
                shooterMotor.setPower(goodVoltageNumberFinder(-0.48f, 12.3f));
            }*/
        }
        if(gamepad2.right_bumper){
            gamepad2.setLedColor(265, 0, 0, 3000);
            intakeSpinning = false;
            intakeMotor.setPower(0);
            shotVelocity = farShot;
            shotMotorOn = true;
            /*if(whichMotorIsDPad == 2) {
                gamepad2.rumble(1000);
                shooterMotor.setPower(DPadNumber);
            }else{
                shooterMotor.setPower(goodVoltageNumberFinder(-0.48f, 12.3f));
            }*/
        }

        if(shotMotorOn){
            shooterMotor.setVelocityPIDFCoefficients(50.0, 0, 109.5, 15.1);
            shooterMotor.setVelocity(shotVelocity);
        }else{
            if(shooterMotor.getVelocity() < 100){
                shooterMotor.setVelocity(0);
            }else {
                shooterMotor.setVelocityPIDFCoefficients(50.0, 0, 109.5, 15.1);
                shooterMotor.setVelocity(-shooterMotor.getVelocity() / 2f);
            }
        }
    }

    private void IntakeMotorManagement(){
        //switch intake motor directinos
        /*if(gamepad2.x){
            if( intakePower == 0.55f ){ intakePower = -0.55f; }
            else{ intakePower = 0.55f; }
        }*/
        //intake motor on and off
        if(intakeSpinning) {
            int multiplierForIntakePower = 1;
            if(gamepad2.x){ multiplierForIntakePower= -1;}
            intakeMotor.setPower(intakePower * multiplierForIntakePower);
            shotMotorOn = false;
        }
        else{intakeMotor.setPower(0);}
        if(gamepad2.right_trigger > 0){intakeSpinning = true;}
        else{intakeSpinning = false;}
    }

    /**
     * Calculates the target RPM based on physics and energy compensation,
     * then sets the motor's velocity using PIDF with dynamic voltage compensation.
     * * @param currentPose The robot's current position (X, Y) from Pedro Pathing (in inches).
     * //@param shooterMotor The DcMotorEx object for your shooter motor.
     * //@param voltageSensor The robot's active VoltageSensor.
     */
    /*public void calculateAndSetShooterVelocity(
            Pose currentPose,
            DcMotorEx shooterMotor,
            VoltageSensor voltageSensor,
            boolean BlueGoal
    )
    {
        if(whichMotorIsDPad == 2){ shooterMotor.setPower(DPadNumber); return; }
        // --- A. PHYSICAL & PID CONSTANTS (CRITICAL: TUNE AND SET THESE) ---

        // PID Gains (Set these after tuning with the separate PID Tuner OpMode)
        final double P_GAIN = 0.00015;
        final double I_GAIN = 0.000001;
        final double D_GAIN = 0.00005;

        // Motor and Encoder Constants
        final double MAX_MOTOR_RPM = 6000.0;
        final double TICKS_PER_REV = 537.7;  // CONFIRM YOUR MOTOR'S Ticks Per Revolution (TPR)
        final double GEAR_RATIO = 1.0;       // Motor shaft rotations per flywheel rotation (e.g., 1.0 for 1:1)

        // Physics and Field Constants (Use METERS for physics calculations)
        final double TARGET_X = (BlueGoal) ? (12*0.0254) : (130*0.0254);  // Target's X field coordinate (meters)
        final double TARGET_Y = 135;              // Target's Y field coordinate (meters)

        final double TARGET_HEIGHT = 1.143;         // Target height above floor (meters)
        final double LAUNCH_HEIGHT = 0.23;        // Artifact exit height above floor (meters)
        final double RAMP_ANGLE_RAD = Math.toRadians(60.0); // Angle of your shot ramp (radians)
        final double FLYWHEEL_RADIUS_M = 0.05;    // Radius of the flywheel (meters)
        final double GRAVITY = 9.81;              // m/s^2
        final double INCHES_TO_METERS = 0.0254;

        // Artifact and Loss Constants (TUNE THESE!)
        final double COEFFICIENT_OF_RESTITUTION = 0.85;
        final double SLIP_COEFFICIENT = 0.98;
        final double CONVERSION_FACTOR = COEFFICIENT_OF_RESTITUTION * SLIP_COEFFICIENT;

        // Voltage Constants
        final double NOMINAL_VOLTAGE = 12.0;

        // --- 1. CALCULATE REQUIRED LAUNCH VELOCITY (v0_required) ---

        // Convert robot pose from inches to meters
        double robotCenterX = currentPose.getX() * INCHES_TO_METERS;
        double robotCenterY = currentPose.getY() * INCHES_TO_METERS;

        // Horizontal distance (X_range) from shot exit point to target center (meters)
        double X_range = Math.sqrt(Math.pow(TARGET_X - robotCenterX, 2) + Math.pow(TARGET_Y - robotCenterY, 2));

        // Vertical height difference (Y_height)
        double Y_height = TARGET_HEIGHT - LAUNCH_HEIGHT;

        // Projectile Motion Denominator (D)
        double D = 2 * Math.pow(Math.cos(RAMP_ANGLE_RAD), 2) * (X_range * Math.tan(RAMP_ANGLE_RAD) - Y_height);

        if (D <= 0) {
            shooterMotor.setVelocity(0); // Impossible shot
            return;
        }

        // Ideal Launch Velocity (v0_required) in m/s
        double v0_squared = (GRAVITY * Math.pow(X_range, 2)) / D;
        double v0_required = Math.sqrt(v0_squared);

        // --- 2. CONVERT v0_required TO MOTOR RPM (With Energy Compensation) ---

        // Compensate v0_required for compression and slip losses (v_flywheel_linear > v0_required)
        double v_flywheel_linear = v0_required / CONVERSION_FACTOR;

        // Angular velocity (rad/s) needed at the flywheel
        double omega_flywheel = v_flywheel_linear / FLYWHEEL_RADIUS_M;

        // Required Motor RPM (Input to PID)
        double targetRPM = (omega_flywheel * (60.0 / (2.0 * Math.PI))) / GEAR_RATIO;

        // --- 3. APPLY PIDF AND VOLTAGE COMPENSATION ---

        // Calculate Target Ticks Per Second (TPS)
        double targetVelocityTps = (targetRPM / 60.0) * TICKS_PER_REV;
        double maxVelocityTps = (MAX_MOTOR_RPM / 60.0) * TICKS_PER_REV;

        // F-Term (Feedforward) Calculation for Voltage Compensation
        // F = (Target Velocity / Max Velocity) * (Nominal Voltage / Current Voltage)
        double currentVoltage = voltageSensor.getVoltage();
        double feedforward = (targetVelocityTps / maxVelocityTps) * (NOMINAL_VOLTAGE / currentVoltage);

        // Clamp feedforward to 1.0 to prevent illegal power commands
        feedforward = Math.min(feedforward, 1.0);

        // Update the PID coefficients with the new dynamic F-term
        shooterMotor.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, feedforward);

        // Set the Target Velocity (The SDK's internal PID controller takes over)
        shooterMotor.setVelocity(targetVelocityTps);
    }
    */
    public void initializationLogic(){
        camera = hardwareMap.get(HuskyLens.class, "camera");
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeSystem");
        intakeServo = hardwareMap.get(Servo.class, "PBTSS");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        intakeServo2 = hardwareMap.get(Servo.class, "PBTSS2");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    boolean areValuesGreaterThan(float value, float axial, float lateral, float yaw){
        if(Math.abs(axial) > value){ return true; }
        if(Math.abs(lateral) > value){ return true; }
        if(Math.abs(yaw) > value){ return true; }
        return  false;
    }
    public void movementLogic(){
        float axial = -gamepad1.left_stick_y;
        float lateral = -gamepad1.left_stick_x;
        float yaw = -gamepad1.right_stick_x;
        if(gamepad1.right_trigger > 0){
            axial = axial * 0.35f;
            lateral = lateral * 0.35f;
            yaw = yaw * 0.35f;
        }
        Pose farShotPoseOG = new Pose(88, 18, Math.toRadians(245));
        Pose closeShotPoseOG = new Pose(82, 135.5, Math.toRadians(180));
            //y = up
        //x = left
        //a = down

        if(gamepad1.y){
            farShotPose = follower.getPose();
        }else if(gamepad1.a){
            closeShotPose = follower.getPose();
        }else if(gamepad1.b){
            farShotPose = farShotPoseOG;
            closeShotPose = closeShotPoseOG;
        }
        if(gamepad1.dpad_up){
            PathChain goToClosestShot = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), closeShotPose))
                    .setLinearHeadingInterpolation(follower.getHeading(), closeShotPose.getHeading())
                    .build();
            follower.followPath(goToClosestShot);
            weAreGoingTo = closeShotPose;
            usingAutoPosition = true;
        } else if(gamepad1.dpad_down){
            PathChain goToFarShot = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), farShotPose))
                    .setLinearHeadingInterpolation(follower.getHeading(), farShotPose.getHeading())
                    .build();
            follower.followPath(goToFarShot);
            weAreGoingTo = farShotPose;
            usingAutoPosition = true;
        }else if(gamepad1.dpad_left){
            PathChain parkBot = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), ParkingPose))
                    .setLinearHeadingInterpolation(follower.getHeading(), ParkingPose.getHeading())
                    .build();
            follower.followPath(parkBot);
            weAreGoingTo = ParkingPose;
            usingAutoPosition = true;
        }
        if(gamepad1.left_trigger > 0){
            follower.breakFollowing();
            follower.startTeleopDrive();
            usingAutoPosition = false;
        }
        if(gamepad1.x) {
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
        }

        if(!usingAutoPosition){
            follower.setTeleOpDrive(axial, lateral, yaw, true);
        }


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        /*double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = (axial - lateral) - yaw;
        double leftBackPower = (axial - lateral) + yaw;
        double rightBackPower = (axial + lateral) - yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        }
        // Send calculated power to wheels.
        if (gamepad1.right_trigger > 0) {
            leftFront.setPower(leftFrontPower / 3);
            rightFront.setPower(rightFrontPower / 3);
            leftBack.setPower(leftBackPower / 3);
            rightBack.setPower(rightBackPower / 3);
        } else {
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }*/
    }
}
