package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.constants.Constants.*;
@Config
public class ArmWithPID {
    private DcMotorEx rotate1, rotate2, extend1, extend2;

    public static double rotp = .0015, roti = ROTI, rotd = .0001, rotf = -.15;
    public static double extp = EXTP, exti = EXTI, extd = EXTD, extf = EXTF;
    //public Telemetry telemetry;
    private PIDController rotatePIDController, extensionPIDController;

    public int rotationTarget = 0;
    private int extensionTarget = 0;
    public static int maxRotation = 9400;
    public static int minRotation = 0;
    public static int maxExtension = -61000;
    public static int minExtension = 1000;


    public ArmWithPID(HardwareMap hardwaremap) {
        rotate1 = hardwaremap.get(DcMotorEx.class, "Arm Rotate 1");
        rotate2 = hardwaremap.get(DcMotorEx.class, "Arm Rotate 2");
        extend1 = hardwaremap.get(DcMotorEx.class, "Arm Extend 1");
        extend2 = hardwaremap.get(DcMotorEx.class, "Arm Extend 2");
        rotatePIDController = new PIDController(rotp, roti, rotd);
        extensionPIDController = new PIDController(extp, exti, extd);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        rotationTarget = getRotationPosition();
//        extensionTarget = getExtensionPosition();

        extend2.setDirection(DcMotorSimple.Direction.REVERSE);
        rotate1.setDirection(DcMotorSimple.Direction.FORWARD);
        rotate2.setDirection(DcMotorSimple.Direction.FORWARD);
        rotate2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotate1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rotate2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void update() {
        rotatePIDController.setPID(rotp, roti, rotd);
        extensionPIDController.setPID(extp, exti, extd);

        double rotPos = rotate2.getCurrentPosition();
        double extPos = getExtensionPosition();

        double rotPID = rotatePIDController.calculate(rotPos, rotationTarget);
        double extPID = extensionPIDController.calculate(extPos, extensionTarget);

        //maybe no tick in degrees
        double rotff = Math.cos(Math.toRadians(rotationTarget/ rot_ticks_in_degree)) *rotf;
        double extff = Math.sin(Math.toRadians(extensionTarget / ext_ticks_in_degree)) * extf;

        double rotPower = Range.clip(rotPID+rotff, -1, 1);
        double extPower = Range.clip(extPID+extff, -1,1);
//        telemetry.addData("ROT TARGET: ", rotPower);
//        telemetry.update();
//        telemetry.addData("Power: ", rotPower);
//        telemetry.update();
        setRotatePower(-rotPower);
        setExtendPower(extPower);
    }

    public void setRotateTarget(int target) {
        rotationTarget = Range.clip(target,minRotation, maxRotation);
    }

    public void setExtensionTarget(int target) {
        extensionTarget = Range.clip(target,minExtension,maxExtension);
    }

    public void manualRotate(double power) {
        setRotateTarget(getRotationPosition()+(int) power*5);
    }

    public void manualExtend(double power) {
        setExtensionTarget(getExtensionPosition()+(int) power*50);
    }


    public void setRotatePower(double power) {
        rotate1.setPower(power);
        rotate2.setPower(power);
    }

    public void setExtendPower(double power) {
        extend1.setPower(power);
        extend2.setPower(power);
    }

    public int getRotationPosition() {
        return rotate2.getCurrentPosition();
    }
    public int getExtensionPosition() {
        return extend2.getCurrentPosition();
    }
}
