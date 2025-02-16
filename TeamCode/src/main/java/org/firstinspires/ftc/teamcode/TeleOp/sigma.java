package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class sigma extends CommandOpMode {

    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public IMU imu;
    public Gamepad g1;

    IMU.Parameters imup;

    @Override
    public void initialize() {
        leftFront = hardwareMap.get (DcMotorEx.class, "fl");
        rightFront = hardwareMap.get (DcMotorEx.class, "fr");
        leftBack = hardwareMap.get (DcMotorEx.class, "bl");
        rightBack = hardwareMap.get (DcMotorEx.class, "br");
        g1 = gamepad1;
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        imup = new IMU.Parameters(
            new RevHubOrientationOnRobot (
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );

        imu.initialize(imup);

    }


    @Override
    public void run() {
        super.run();
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double bHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-bHeading) - y * Math.sin(-bHeading);
        double rotY = x * Math.sin(-bHeading) + y * Math.cos(-bHeading);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flp = (rotY + rotX + rx) / denominator;
        double frp = (rotY - rotX - rx) / denominator;
        double blp = (rotY - rotX + rx) / denominator;
        double brp = (rotY + rotX - rx) / denominator;

        leftFront.setPower(flp);
        rightFront.setPower(frp);
        leftBack.setPower(blp);
        rightBack.setPower(brp);


    }
}