package org.firstinspires.ftc.teamcode.Subsystems;
import static org.firstinspires.ftc.teamcode.constants.Constants.*;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.UsefulStuff.PinpointOdo;
@Config
public class FieldCentricDrive extends SubsystemBase {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public static double powerMultiplier = .8;
    PinpointOdo odo;

    public FieldCentricDrive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        //odo setup
        odo = hardwareMap.get(PinpointOdo.class,"odo");
        odo.setOffsets(pinpointXOffset,pinpointYOffset);
        odo.setEncoderResolution(PinpointOdo.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(PinpointOdo.EncoderDirection.REVERSED,
                PinpointOdo.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        //often good. might not need to or with different motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double x, double y, double rx) {
        odo.update();



        Pose2D pos = odo.getPosition();
        double botHeading = pos.getHeading(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) + y * Math.sin(-botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        rotY = -rotY;
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        // Set the motor powers
        leftFront.setPower(frontLeftPower*powerMultiplier);
        leftBack.setPower(backLeftPower*powerMultiplier);
        rightFront.setPower(frontRightPower*powerMultiplier);
        rightBack.setPower(backRightPower*powerMultiplier);
    }

    public void resetIMU() {
        odo.resetPosAndIMU();
    }

    public Pose2D getPosition() {
        return odo.getPosition();
    }

    public void setPowerMultiplier(double power) {
        powerMultiplier = power;
    }

}
