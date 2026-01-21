package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "cameratest")
public class cameratest extends LinearOpMode {
    private HuskyLens huskylens;

    /**
     * This OpMode illustrates how to use the DFRobot HuskyLens.
     *
     * The HuskyLens is a Vision Sensor with a built-in object detection model. It can
     * detect a number of predefined objects and AprilTags in the 36h11 family, can
     * recognize colors, and can be trained to detect custom objects. See this website for
     * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
     *
     * This sample illustrates how to detect AprilTags, but can be used to detect other types
     * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
     * a name of "huskylens".
     */
    @Override
    public void runOpMode() {
        ElapsedTime myElapsedTime;
        HuskyLens.Block[] myHuskyLensBlocks;
        HuskyLens.Block myHuskyLensBlock;

        huskylens = hardwareMap.get(HuskyLens.class, "camera");

        // Put initialization blocks here.
        telemetry.addData(">>", huskylens.knock() ? "Touch start to continue" : "Problem communicating with HuskyLens");
        huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.update();
        myElapsedTime = new ElapsedTime();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (myElapsedTime.seconds() >= 1) {
                    myElapsedTime.reset();
                    myHuskyLensBlocks = huskylens.blocks();
                    telemetry.addData("Block count", JavaUtil.listLength(myHuskyLensBlocks));
                    for (HuskyLens.Block myHuskyLensBlock_item : myHuskyLensBlocks) {
                        myHuskyLensBlock = myHuskyLensBlock_item;
                        telemetry.addData("Block", "id=" + myHuskyLensBlock.id + " size: " + myHuskyLensBlock.width + "x" + myHuskyLensBlock.height + " position: " + myHuskyLensBlock.x + "," + myHuskyLensBlock.y);
                    }
                    telemetry.update();
                }
            }
        }
    }
}
