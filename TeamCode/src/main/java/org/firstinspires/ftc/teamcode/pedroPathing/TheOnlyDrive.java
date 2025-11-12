package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TheOnlyDriveNew")
public class TheOnlyDrive extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor rightBack;
    private Follower follower;
    private DcMotorEx shooterMotor;
    private DcMotor leftFront;
    private float DPadNumber = 0f;
    private Servo intakeServo;
    private DcMotor leftBack;
    float intakePower = 1;
    Timer shotTimer = new Timer();
    enum IntakeServoPos{
        Standby,
        Holding,
        SendToShooting
    }
    IntakeServoPos currentServoState = IntakeServoPos.Standby;
    private DcMotor intakeMotor;
    TelemetryManager telemetryM;
    private final Pose autoEndPose = new Pose(39, 33, Math.toRadians(90));
    boolean scoringInBlueGoal = true;
    int whichMotorIsDPad = 0;
    //leftyimu
    //middleximu
    //rightyimu
    boolean intakeSpinning = false;
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        float axial = 0;
        float lateral = 0;
        //1 ball .2491
        //2 ball .4493
        //3 ball .5495
        //send to shot .2991
        float yaw = 0;
        double max = 0;
        runtime = new ElapsedTime();
        initializationLogic();
        FollowerInit();
        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            // Pre-run
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                // OpMode loop
                follower.update();
                telemetryM.update();
                movementLogic(runtime, axial, lateral, yaw, max);
                DPadNumberManagement();
                IntakeMotorManagement();
                if(gamepad1.a){scoringInBlueGoal = !scoringInBlueGoal;}
                ServoManagement();
                //make a shot, servo sending balls to shot system

                telemetryData();
            }
        }
    }
    public void telemetryData(){
        telemetry.addData("Intake Motor Position: ", intakeServo.getPosition());
        telemetry.addData("D-Pad Number: ", DPadNumber);
        telemetry.addLine("0 = Intake Servo, 1 = Intake Motor, 2 = Shot Motor");
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Scoring In Blue Goal: ", scoringInBlueGoal);
        telemetry.update();
    }
    public VoltageSensor getBatteryVoltage() {
        double maxVoltage = 0;
        VoltageSensor finalVolter = null;
        // Iterate over all VoltageSensors available in the hardware map
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > maxVoltage) {
                maxVoltage = voltage;
                finalVolter = sensor;
            }
        }

        return finalVolter;
    }
    private void FollowerInit(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(autoEndPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }
    private void ServoManagement(){
        if(gamepad2.x){
            currentServoState = IntakeServoPos.SendToShooting;
            shotTimer.resetTimer();
            int waitTime = 1500;
            float intakeFinalPos = 0.25f;
            calculateAndSetShooterVelocity(follower.getPose(), shooterMotor, getBatteryVoltage(), scoringInBlueGoal);
            while(shotTimer.getElapsedTime() < waitTime){
                if(gamepad2.left_trigger>0){
                    currentServoState = IntakeServoPos.Holding;
                    waitTime = 0;
                    intakeFinalPos = 0.5495f;
                }
            }
            intakeServo.setPosition(intakeFinalPos);
        }
        if(gamepad2.b){
            if(currentServoState == IntakeServoPos.Holding){
                intakeServo.setPosition(0.8);
                currentServoState = IntakeServoPos.Standby;
                //take in 1
            }else if(currentServoState == IntakeServoPos.Standby){
                intakeServo.setPosition(0.5495f);
                currentServoState = IntakeServoPos.Holding;
            }else if(currentServoState == IntakeServoPos.SendToShooting){
                currentServoState = IntakeServoPos.Standby;
                intakeServo.setPosition(0.8);
            }
        }
    }
    private void DPadNumberManagement(){
        DPadNumChanging();
        if(gamepad2.y){
            if(whichMotorIsDPad == 2){whichMotorIsDPad = -1;}
            whichMotorIsDPad++;
        }
        switch (whichMotorIsDPad){
            case 0:
                intakeServo.setPosition(DPadNumber);
                break;
            case 1:
                intakePower = DPadNumber;
                break;
            case 2:
                shooterMotor.setPower(DPadNumber);
                break;
            default:
                whichMotorIsDPad = 0;
                break;
        }
    }
    private void IntakeMotorManagement(){
        //switch intake motor directinos
        if(gamepad2.a){
            if( intakePower == 0.5 ){ intakePower = -0.5f; }
            else{ intakePower = 0.5f; }
        }
        //intake motor on and off
        if(intakeSpinning) {intakeMotor.setPower(intakePower);}
        else{intakeMotor.setPower(0);}
        if(gamepad2.right_trigger > 0){intakeSpinning = true;}
        else{intakeSpinning = false;}
    }
    private void DPadNumChanging(){
        if(gamepad2.dpad_up){
            DPadNumber += 0.1f;
        }
        if(gamepad2.dpad_down){
            DPadNumber -= 0.1f;
        }
        if(gamepad2.dpad_right){
            DPadNumber += 0.01f;
        }
        if(gamepad2.dpad_left){
            DPadNumber -= 0.01f;
        }
    }
    /**
     * Calculates the target RPM based on physics and energy compensation,
     * then sets the motor's velocity using PIDF with dynamic voltage compensation.
     * * @param currentPose The robot's current position (X, Y) from Pedro Pathing (in inches).
     * @param shooterMotor The DcMotorEx object for your shooter motor.
     * @param voltageSensor The robot's active VoltageSensor.
     */
    public void calculateAndSetShooterVelocity(
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
        final double TARGET_X = (BlueGoal) ? 12 : 130;              // Target's X field coordinate (meters)
        final double TARGET_Y = 135;              // Target's Y field coordinate (meters)

        final double TARGET_HEIGHT = 1.143;         // Target height above floor (meters)
        final double LAUNCH_HEIGHT = 0.15;        // Artifact exit height above floor (meters)
        final double RAMP_ANGLE_RAD = Math.toRadians(30.0); // Angle of your shot ramp (radians)
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
    public void initializationLogic(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeSystem");
        intakeServo = hardwareMap.get(Servo.class, "PBTSS");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void movementLogic(ElapsedTime runtime, float axial, float lateral, float yaw, double max){
        axial = -gamepad1.left_stick_y;
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;
        if(gamepad1.right_trigger>0){
            axial = axial / 3;
            lateral = lateral / 3;
            yaw = yaw / 3;
        }
        follower.setTeleOpDrive((double) axial, (double) lateral, (double) yaw, false);
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        /*leftFrontPower = axial + lateral + yaw;
        rightFrontPower = (axial - lateral) - yaw;
        leftBackPower = (axial - lateral) + yaw;
        rightBackPower = (axial + lateral) - yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
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
