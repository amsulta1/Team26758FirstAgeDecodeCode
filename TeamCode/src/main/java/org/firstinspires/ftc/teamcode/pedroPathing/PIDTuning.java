package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.*;

/**
 * Robust PID Tuning OpMode using simple static finals for guaranteed compilation.
 * To Tune: Edit the static final values below, SAVE, BUILD (recompile), and RUN the OpMode again.
 */
@TeleOp(name = "PID_Tuning_FINAL", group = "Tuning")
public class PIDTuning extends LinearOpMode {
    DcMotorEx shooterMotor;
    int whichVarChanging = 0;
    float VELOCITY = 0;
    float savedVelocity = 1300f;
    PIDFController controller;
    boolean shooterMotorOn = false;
    ElapsedTime timerForTargetSpeed = new ElapsedTime();
    float TICKSINDEGREES = 700.0f / 180.0f;
    boolean activelyTiming = false;
    //double P = 50, I = 0, D = 109.5, F = 15.1;
    double P = 50.0f, I = 0f, D = 150.1f, F = 10.0f;
    double finalTimeForShotTiming = -1f;
    //short shot 1110
    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //P = shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
        //I = shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
        //D = shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
        //F = shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;
        waitForStart();

        while (opModeIsActive()) {
            DPadControls();
            PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F);
            shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
            shooterMotor.setVelocity(VELOCITY);
            //PIDFCoefficients pids = new PIDFCoefficients(P, I, D, F);
            //shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pids);
            if(activelyTiming){
                if(shooterMotor.getVelocity() >= VELOCITY){
                    finalTimeForShotTiming = timerForTargetSpeed.milliseconds() - 200;
                    activelyTiming = false;
                }
            }
            if(gamepad1.a){
                if(whichVarChanging >= 4){
                    whichVarChanging = -1;
                }
                whichVarChanging++;
                sleep(200);
            }
            if(gamepad1.y){
                shooterMotorOn = !shooterMotorOn;
                sleep(200);
            }
            if(gamepad1.x){
                savedVelocity = VELOCITY;
                sleep(200);
            }
            if(gamepad1.left_trigger > 0){
                VELOCITY = 0;
                while(shooterMotor.getVelocity() > 100){
                    shooterMotor.setVelocity(-shooterMotor.getVelocity()/2f);
                }
                sleep(200);
            }
            if(gamepad1.right_bumper){
                activelyTiming = true;
                timerForTargetSpeed.reset();
                finalTimeForShotTiming = -1f;
                VELOCITY = savedVelocity;
                sleep(200);
            }
            telemetry.addData("Current Coefficients", shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
            telemetry.addLine("0 = P \n1 = I \n2 = D \n3 = F \n4 = VELOCITY");
            telemetry.addData("Currently Changing: ", whichVarChanging);
            telemetry.addData("P = ", P);
            telemetry.addData("I = ", I);
            telemetry.addData("D = ", D);
            telemetry.addData("F = ", F);
            telemetry.addData("Target Velocity: ", VELOCITY);
            telemetry.addData("Current Motor Velocity: ", shooterMotor.getVelocity());
            if(finalTimeForShotTiming > 0f){
                telemetry.addData("Recovery Time in Milliseconds: ", finalTimeForShotTiming);
            }
            telemetry.update();

        }
    }
    private void DPadControls(){
        float DPadNumber = 0;
        float multiplier = 100f;
        if(whichVarChanging == 4){
            multiplier = 10000;
        }

        if(gamepad1.dpad_up){
            DPadNumber += 0.001f * multiplier;
            sleep(90);
        }
        if(gamepad1.dpad_down){
            DPadNumber -= 0.001f * multiplier;
            sleep(90);
        }
        if(gamepad1.dpad_right){
            DPadNumber += 0.0001f * multiplier;
            sleep(90);
        }
        if(gamepad1.dpad_left){
            DPadNumber -= 0.0001f * multiplier;
            sleep(90);
        }
        switch (whichVarChanging){
            case 0:
                P = P + DPadNumber;
                break;
            case 1:
                I = I + DPadNumber;
                break;
            case 2:
                D = D + DPadNumber;
                break;
            case 3:
                F = F + DPadNumber;
                break;
            case 4:
                VELOCITY = VELOCITY + DPadNumber;
            default:
                break;
        }
    }
}
