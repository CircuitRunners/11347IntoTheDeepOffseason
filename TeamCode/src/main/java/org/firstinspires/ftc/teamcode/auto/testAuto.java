package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Autonomous
public class testAuto extends OpMode {
    private Follower follower;
    private int pathState = 0;
    private Pose startPos = new Pose(10, 62.5, Math.toRadians(0));
    private Pose endPos = new Pose(32.8, 62.5, Math.toRadians(0));


    private PathChain testPath;
    public void buildPaths() {
        testPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(endPos)))
                .setConstantHeadingInterpolation(endPos.getHeading())
                .build();

    }



    @Override
    public void init() {

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPos);
        //follower.setMaxPower(1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        buildPaths();
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(testPath);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("Current Path #:", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toRadians(follower.getPose().getHeading()));
        telemetry.update();
    }

}
