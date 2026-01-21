package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPositioning")
public class ServoPositioning extends LinearOpMode {
    Servo intakeOG;
    Servo intakeSecondary;
    float mainVal = 1f;
    float secondVal = 0f;
    //holdin main: .91 second .13     second first, main afterward
    //shooting 3  main .82, it should be holding first though.
    //shooting 2  main 0.6 it should be from shooting 3.
    //shooting 1 main .52 secondary .62 from shooting 2 both of them together.
    boolean usingMain = true;

    @Override
    public void runOpMode() {
        intakeOG = hardwareMap.get(Servo.class, "PBTSS");
        intakeSecondary = hardwareMap.get(Servo.class, "PBTSS2");
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                if (gamepad1.y) {
                    usingMain = !usingMain;
                    sleep(200);
                }
                DPadControls();
                intakeOG.setPosition(mainVal);
                intakeSecondary.setPosition(secondVal);
                if (usingMain) {
                    telemetry.addLine("Using Main Servo");
                } else {
                    telemetry.addLine("Using Secondar Servo");
                }
                telemetry.addData("Main Servo Position: ", mainVal);
                telemetry.addData("Secondary Servo Position: ", secondVal);
                telemetry.update();
            }
        }
    }
    //0.01 is open (secondary) 0.69 is sweet spot for main.
    // open is 0.01 second     and 0.75 main

    //holding is main 0.6 first,  0.1 secondary in both order and servo name
    //shoot 3 = main same,  secondary 0.2, main 0.69
    //shoot 2 = main same      second 0.39
    //shoot 1 = main same      second 0.59
    private void DPadControls() {
        float DPadNumber = 0;
        float multiplier = 100f;

        if (gamepad1.dpad_up) {
            DPadNumber += 0.001f * multiplier;
            sleep(90);
        }
        if (gamepad1.dpad_down) {
            DPadNumber -= 0.001f * multiplier;
            sleep(90);
        }
        if (gamepad1.dpad_right) {
            DPadNumber += 0.0001f * multiplier;
            sleep(90);
        }
        if (gamepad1.dpad_left) {
            DPadNumber -= 0.0001f * multiplier;
            sleep(90);
        }
        if (usingMain) {
            mainVal += DPadNumber;
        } else {
            secondVal += DPadNumber;

        }
    }
}
   