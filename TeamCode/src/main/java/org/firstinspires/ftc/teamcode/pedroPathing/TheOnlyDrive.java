package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TheOnlyDriveNew")
public class TheOnlyDrive extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private Servo intakeServo;
    private DcMotor leftBack;
    float intakePower = 1;
    enum IntakeServoPos{
        Standby,
        Holding,
        SendToShooting
    }
    IntakeServoPos currentServoState = IntakeServoPos.Standby;
    private DcMotor intakeMotor;
    double leftFrontPower;
    double rightFrontPower;
    double rightBackPower;
    double leftBackPower;
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
        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                movementLogic(runtime, axial, lateral, yaw, max);
                if(gamepad2.a){
                    if( intakePower == 0.5 ){ intakePower = -0.5f; }
                    else{ intakePower = 0.5f; }
                }
                if(intakeSpinning) {
                    intakeMotor.setPower(intakePower);
                }
                else{
                    intakeMotor.setPower(0);
                }
                if(gamepad2.right_trigger > 0){
                    intakeSpinning = true;
                }//pushballstoshotsytem
                else{
                    intakeSpinning = false;
                }
                if(gamepad2.x){
                    currentServoState = IntakeServoPos.SendToShooting;
                    intakeServo.setPosition(0.25);
                }
                if(gamepad2.b){
                    if(currentServoState == IntakeServoPos.Holding){
                        intakeServo.setPosition(0.8);
                        currentServoState = IntakeServoPos.Standby;
                        //take in 1
                    }else if(currentServoState == IntakeServoPos.Standby){
                        intakeServo.setPosition(0.5);
                        currentServoState = IntakeServoPos.Holding;
                    }else if(currentServoState == IntakeServoPos.SendToShooting){
                        currentServoState = IntakeServoPos.Standby;
                        intakeServo.setPosition(0.8);
                    }
                }
                telemetryData();
            }
        }
    }
    public void telemetryData(){
        telemetry.addData("Intake Motor Position: ", intakeServo.getPosition());
        telemetry.update();
    }
    public void initializationLogic(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeSystem");
        intakeServo = hardwareMap.get(Servo.class, "PBTSS");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
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
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower = axial + lateral + yaw;
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
        }
    }
}
