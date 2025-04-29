package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmWithPID;
import org.firstinspires.ftc.teamcode.Subsystems.Diffy;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class specimenAuto extends OpMode {
    private Follower follower;
    private int pathState = 0;
    private boolean thirdSample = false;
    private ArmWithPID arm;
    private Diffy diffy;

    private Pose startPos = new Pose(10, 62.5, Math.toRadians(0));
    private Pose preloadPos = new Pose(32.8, 62.5, Math.toRadians(0));
    private Pose sample1GrabPos = new Pose(64, 24, Math.toRadians(0));//90 //25
    private Point sample1GrabCP1 = new Point(34, 12.5);
    private Point sample1GrabCP2 = new Point(59, 50);//48
    private Pose sample1PlacePos = new Pose(18, 24);//, Math.toRadians(90)
    private Pose sample2GrabPos = new Pose(60, 15.5);//, Math.toRadians(90)
    private Point sample2GrabCP = new Point(62, 35);
    private Pose sample2PlacePos = new Pose(17, 15.5);//, Math.toRadians(90)
    private Pose sample3GrabPos = new Pose(41.5, 13);//, Math.toRadians(90)
    private Pose sample3PlacePos = new Pose(20, 13, Math.toRadians(0));
    private Pose specimenGrabPos = new Pose(15.5,34); //, Math.toRadians(225)
    private Point specimen1GrabCP = new Point(45, 34);
    private Pose specimen1PlacePos = new Pose(32.8, 61, Math.toRadians(0));
    private Pose specimen2PlacePos = new Pose(32.8, 62, Math.toRadians(0));
    //private Pose specimen3PlacePos = new Pose(34, 61, Math.toRadians(0));
    private Pose parkPos = new Pose(10, 10, Math.toRadians(0));

    private PathChain preload, sample1Grab, sample1Place, sample2Grab, sample2Place, specimen1Grab, specimen1Place, specimen2Grab, specimen2Place, park;
    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(preloadPos)))
                .setConstantHeadingInterpolation(preloadPos.getHeading())
                .build();
        sample1Grab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPos), sample1GrabCP1, sample1GrabCP2, new Point(sample1GrabPos)))
                .setConstantHeadingInterpolation(sample1GrabPos.getHeading())
                .addPath(new BezierLine(new Point(sample1GrabPos), new Point(sample1PlacePos)))
                .setConstantHeadingInterpolation(sample1PlacePos.getHeading())
                .build();
        sample2Grab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample1PlacePos), sample2GrabCP, new Point(sample2GrabPos)))
                .setConstantHeadingInterpolation(sample2GrabPos.getHeading())
                .addPath(new BezierLine(new Point(sample2GrabPos), new Point(sample2PlacePos)))
                .setConstantHeadingInterpolation(sample2PlacePos.getHeading())
                .build();
//        sample3Grab = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(sample2PlacePos), new Point(sample3GrabPos)))
//                .setConstantHeadingInterpolation(sample3GrabPos.getHeading())
//                .build();
//        sample3Place = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(sample3GrabPos), new Point(sample3PlacePos)))
//                .setConstantHeadingInterpolation(sample3PlacePos.getHeading())
//                .build();
//        specimen1Grab = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(sample3PlacePos), specimen1GrabCP, new Point(specimenGrabPos)))
//                .setConstantHeadingInterpolation(specimenGrabPos.getHeading())
//                .build();
        specimen1Grab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample2PlacePos), specimen1GrabCP, new Point(specimenGrabPos)))
                .setConstantHeadingInterpolation(specimenGrabPos.getHeading())
                .build();
//        specimen1GrabStraight = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(sample2PlacePos), new Point(specimenGrabPos)))
//                .setLinearHeadingInterpolation(sample2PlacePos.getHeading(), specimenGrabPos.getHeading())
//                .build();
        specimen1Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenGrabPos), new Point(specimen1PlacePos)))
                .setLinearHeadingInterpolation(specimenGrabPos.getHeading(), specimen1PlacePos.getHeading())
                .build();
        specimen2Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1PlacePos), new Point(specimenGrabPos)))
                .setLinearHeadingInterpolation(specimen1PlacePos.getHeading(), specimenGrabPos.getHeading())
                .build();
        specimen2Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenGrabPos), new Point(specimen2PlacePos)))
                .setLinearHeadingInterpolation(specimenGrabPos.getHeading(), specimen2PlacePos.getHeading())
                .build();
//        specimen3Grab = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(specimen2PlacePos), new Point(specimenGrabPos)))
//                .setConstantHeadingInterpolation(specimenGrabPos.getHeading())
//                .build();
//        specimen3Place = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(specimenGrabPos), new Point(specimen3PlacePos)))
//                .setConstantHeadingInterpolation(specimen3PlacePos.getHeading())
//                .build();
//        park = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(specimen3PlacePos), new Point(parkPos)))
//                .setConstantHeadingInterpolation(parkPos.getHeading())
//                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2PlacePos), new Point(parkPos)))
                .setConstantHeadingInterpolation(parkPos.getHeading())
                .build();
    }


    public void pathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preload,true);
                arm.rotateFull();
                pathState = 1;
                break;
            case 1:
                if(!follower.isBusy()) {
                    diffy.rotateDiffyDown();
                    pathState = 2;
                }
                break;
            case 2:
                //Code to wait
                diffy.openClaw();
                diffy.rotateDiffyUp();
                arm.rotateRest();
                follower.followPath(sample1Grab);
                pathState = 3;
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(sample1Place);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(sample2Grab);
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(sample2Place);
                    if (thirdSample) {
                        pathState = 6;
                    } else {
                        pathState = 9;
                    }
                }
                break;
                //Third sample
            case 6:
                //path: sample three grab
                pathState = 7;
                break;
            case 7:
                //path: sample three place
                pathState = 8;
                break;
            case 8:
                //path: specimen one grab from spec three
                //Arm wall pos
                //claw open
                pathState = 10;
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(specimen1Grab);
                    arm.rotateWall();
                    diffy.openClaw();
                    pathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    diffy.closeClaw();
                    pathState = 11;
                }
                break;
            case 11:
                //Code to wait
                arm.rotateFull();
                diffy.rotateDiffyUp();
                follower.followPath(specimen1Place);
                pathState = 12;
                break;
            case 12:
                if(!follower.isBusy()) {
                    diffy.rotateDiffyDown();
                    pathState = 13;
                }
                break;
            case 13:
                //Code to wait
                diffy.openClaw();
                arm.rotateWall();
                follower.followPath(specimen2Grab);
                pathState = 14;
                break;
            case 14:
                if(!follower.isBusy()) {
                    diffy.closeClaw();
                }
                pathState = 15;
                break;
            case 15:
                //Code to wait
                arm.rotateFull();
                diffy.rotateDiffyUp();
                follower.followPath(specimen2Place);
                pathState = 16;
                break;
            case 16:
                if(!follower.isBusy()) {
                    diffy.rotateDiffyDown();
                    if (thirdSample) {
                        pathState = 17;
                    } else {
                        pathState = 22;
                    }
                }
                break;
            case 17:
                //claw open
                //Arm wall pos
                //path: spec three grab
                pathState = 18;
                break;
            case 18:
                //claw close
                pathState = 19;
                break;
            case 19:
                //arm rotate full
                //claw rotate top
                //path: specimen three place
                pathState = 20;
                break;
            case 20:
                //claw rotate bottom
                pathState = 21;
                break;
            case 21:
                //claw open
                //mechanism reset
                //path: park from spec three
                pathState = 23;
                break;
            case 22:
                //Code to wait
                diffy.openClaw();
                diffy.rotateDiffyUp();
                arm.rotateRest();
                follower.followPath(park);
                pathState = 23;
            default:
                if(!follower.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }

    }

    @Override
    public void init() {
        arm = new ArmWithPID(hardwareMap);
        diffy = new Diffy(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPos);
        //follower.setMaxPower(1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        buildPaths();
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();
        arm.update();

        telemetry.addData("Current Path #:", pathState);
        telemetry.addData("Rotation Position:", arm.getRotationPosition());
        telemetry.addData("Extension Position:", arm.getExtensionPosition());
        telemetry.addData("Diffy Left Pos:", diffy.leftServoPosition());
        telemetry.addData("Diffy Right Pos:", diffy.rightServoPosition());
        telemetry.addData("Is Claw Open?:", diffy.isClawOpen());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toRadians(follower.getPose().getHeading()));
        telemetry.update();
    }

}
