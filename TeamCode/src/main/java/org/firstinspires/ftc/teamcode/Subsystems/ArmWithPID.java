package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.constants.Constants.*;

public class ArmWithPID {
    private DcMotorEx rotate1, rotate2, extend1, extend2;

    private double rotp = ROTP, roti = ROTI, rotd = ROTD, rotf = ROTF;
    private double extp = EXTP, exti = EXTI, extd = EXTD, extf = EXTF;

    private PIDController rotatePIDController, extensionPIDController;

    private int rotationTarget = 0;
    private int extensionTarget = 0;


    public ArmWithPID(HardwareMap hardwaremap) {
        rotate1 = hardwaremap.get(DcMotorEx.class, "Arm Rotate 1");
        rotate2 = hardwaremap.get(DcMotorEx.class, "Arm Rotate 2");
        extend1 = hardwaremap.get(DcMotorEx.class, "Arm Extend 1");
        extend2 = hardwaremap.get(DcMotorEx.class, "Arm Extend 2");
        rotatePIDController = new PIDController(rotp, roti, rotd);
        extensionPIDController = new PIDController(extp, exti, extd);

        rotationTarget = getRotationPosition();
        extensionTarget = getExtensionPosition();

        extend2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        rotatePIDController.setPID(rotp, roti, rotd);
        extensionPIDController.setPID(extp, exti, extd);

        double rotPos = getRotationPosition();
        double extPos = getExtensionPosition();

        double rotPID = rotatePIDController.calculate(rotPos, rotationTarget);
        double extPID = extensionPIDController.calculate(extPos, extensionTarget);

        double rotff = Math.sin(Math.toRadians(rotationTarget / rot_ticks_in_degree)) *rotf;
        double extff = Math.sin(Math.toRadians(extensionTarget / ext_ticks_in_degree)) * extf;

        double rotPower = Range.clip(rotPID+rotff, -1, 1);
        double extPower = Range.clip(extPID+extff, -1,1);

        setRotatePower(rotPower);
        setExtendPower(extPower);
    }

    public void setRotateTarget(int target) {
        rotationTarget = target;
    }

    public void setExtensionTarget(int target) {
        extensionTarget = target;
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
        return rotate1.getCurrentPosition();
    }
    public int getExtensionPosition() {
        return extend1.getCurrentPosition();
    }
}
