package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@TeleOp(name = "PIDTuning")
public class PIDTuning extends LinearOpMode {
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    private PIDFController controller;
    public static int target = 0;
    private DcMotor motor;
    private final double ticks_in_degrees = 700/180.0;
    @Override
    public void runOpMode() {
        PIDFCoefficients setter = new PIDFCoefficients(p, i, d, f);
        controller = new PIDFController(setter);
        motor = hardwareMap.get(DcMotor.class, "Shooter");
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                setter = new PIDFCoefficients(p, i, d, f);
                controller.setCoefficients(setter);
                int Pos = motor.getCurrentPosition();
                controller.setTargetPosition(target);
                controller.updatePosition(Pos);
                motor.setPower(controller.run());
            }
        }
    }
}
