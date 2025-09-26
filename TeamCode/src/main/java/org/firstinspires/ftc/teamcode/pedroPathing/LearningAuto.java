package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "TestingAuto123")
public class LearningAuto extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(96, 8, Math.toRadians(90));
    private final Pose endPose = new Pose(96, 136, Math.toRadians(180));
    Path fullAutoPath;
    public void buildPaths(){
        fullAutoPath = new Path(new BezierLine(startPose, endPose));
        fullAutoPath.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
    }
    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            follower.followPath(fullAutoPath);
            while (opModeIsActive()) {
                // OpMode loop
            }
        }
    }
}
