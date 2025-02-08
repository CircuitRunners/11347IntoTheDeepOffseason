package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private DcMotorEx rotate1, rotate2, extend1, extend2;
    public static final double rotateMultiplier = 1;//If we want less power
    public static final double extendMultiplier = 1;//If we want less power


    public Arm(HardwareMap hardwaremap) {
        rotate1 = hardwaremap.get(DcMotorEx.class, "Arm Rotate 1");
        rotate2 = hardwaremap.get(DcMotorEx.class, "Arm Rotate 2");
        extend1 = hardwaremap.get(DcMotorEx.class, "Arm Extend 1");
        extend2 = hardwaremap.get(DcMotorEx.class, "Arm Extend 2");
        //Might be needed
        extend2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void manualRotate(double power) {

        rotate1.setPower(power*rotateMultiplier);
        rotate2.setPower(power*rotateMultiplier);
    }

    public void manualExtend(double power) {
        extend1.setPower(power*extendMultiplier);
        extend2.setPower(power*extendMultiplier);
    }
}
