package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.ftc.localization.Encoder;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

/**
 * Encoder Diagnostic Tool
 *
 * This helps you figure out which encoder is which and if they're going the right direction.
 *
 * INSTRUCTIONS:
 * 1. Run this OpMode
 * 2. Manually spin each wheel/encoder one at a time
 * 3. Watch which value changes on telemetry
 * 4. Note the direction (positive or negative)
 * 5. Use this info to fix your Constants.java encoder mapping
 *
 * WHAT TO TEST:
 * - Spin LEFT parallel odometry wheel → "Left Encoder" should change
 * - Spin RIGHT parallel odometry wheel → "Right Encoder" should change
 * - Spin PERPENDICULAR (strafe) odometry wheel → "Strafe Encoder" should change
 * - Push robot FORWARD → Left and Right should increase
 * - Push robot RIGHT → Strafe should increase
 * - Rotate robot COUNTERCLOCKWISE → Left should decrease, Right should increase (or vice versa)
 */
@TeleOp(name = "Encoder Diagnostic", group = "Tuning")
public class EncoderDiagnostics extends OpMode {

    private DcMotorEx leftEncoder;
    private DcMotorEx rightEncoder;
    private DcMotorEx strafeEncoder;

    private int leftStart = 0;
    private int rightStart = 0;
    private int strafeStart = 0;

    @Override
    public void init() {
        // Setup FTC Dashboard
        //telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // Get encoders from the hardware map names in Constants.java
        // These are the motor ports your odometry pods are plugged into
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        strafeEncoder = hardwareMap.get(DcMotorEx.class, "rightBack");

        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        strafeEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

        // Record starting positions
        leftStart = leftEncoder.getCurrentPosition();
        rightStart = rightEncoder.getCurrentPosition();
        strafeStart = strafeEncoder.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Now manually move each encoder and watch values");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get current encoder positions (raw ticks)
        int leftRaw = leftEncoder.getCurrentPosition();
        int rightRaw = rightEncoder.getCurrentPosition();
        int strafeRaw = strafeEncoder.getCurrentPosition();

        // Calculate changes from start
        int leftDelta = leftRaw - leftStart;
        int rightDelta = rightRaw - rightStart;
        int strafeDelta = strafeRaw - strafeStart;

        // Display raw encoder values
        telemetry.addData("=== RAW ENCODER TICKS ===", "");
        telemetry.addData("Left Encoder (frontLeftMotor)", leftRaw);
        telemetry.addData("Right Encoder (frontRightMotor)", rightRaw);
        telemetry.addData("Strafe Encoder (backLeftMotor)", strafeRaw);
        telemetry.addLine();

        telemetry.addData("=== CHANGE FROM START ===", "");
        telemetry.addData("Left Delta", leftDelta);
        telemetry.addData("Right Delta", rightDelta);
        telemetry.addData("Strafe Delta", strafeDelta);
        telemetry.addLine();

        // Provide diagnostic guidance
        telemetry.addData("=== DIAGNOSTIC TESTS ===", "");
        telemetry.addLine("1. Spin LEFT odometry wheel:");
        telemetry.addLine("   → Only 'Left Delta' should change");
        telemetry.addLine();
        telemetry.addLine("2. Spin RIGHT odometry wheel:");
        telemetry.addLine("   → Only 'Right Delta' should change");
        telemetry.addLine();
        telemetry.addLine("3. Spin STRAFE odometry wheel:");
        telemetry.addLine("   → Only 'Strafe Delta' should change");
        telemetry.addLine();
        telemetry.addLine("4. Push robot FORWARD 12 inches:");
        telemetry.addLine("   → Left and Right should both increase");
        telemetry.addLine("   → Should get ~500-2000 ticks");
        telemetry.addLine();
        telemetry.addLine("5. Push robot RIGHT 12 inches:");
        telemetry.addLine("   → Strafe should increase");
        telemetry.addLine("   → Should get ~500-2000 ticks");
        telemetry.addLine();

        // Calculate what the conversion should be (rough estimate)
        if (Math.abs(leftDelta) > 100) {
            double estimatedForward = 12.0 / Math.abs(leftDelta);
            telemetry.addData("Estimated forwardTicksToInches", "%.9f", estimatedForward);
            telemetry.addLine("(if you pushed 12 inches forward)");
        }
        if (Math.abs(strafeDelta) > 100) {
            double estimatedStrafe = 12.0 / Math.abs(strafeDelta);
            telemetry.addData("Estimated strafeTicksToInches", "%.9f", estimatedStrafe);
            telemetry.addLine("(if you pushed 12 inches right)");
        }

        telemetry.addLine();
        telemetry.addData("Press BACK to reset", "starting positions");

        // Reset on back button
        if (gamepad1.back) {
            leftStart = leftRaw;
            rightStart = rightRaw;
            strafeStart = strafeRaw;
        }

        telemetry.update();
    }
}
