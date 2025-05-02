package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.UsefulStuff.Constants.*;
@Config
public class ArmWithPID {
    private DcMotorEx rotate1, rotate2, extend1, extend2;

    public static double rotp = .0017, roti = 0, rotd = 0.00007, rotf = -.10;
    public static double extp = 0.001, exti = 0, extd = 0.00001, extf = 0;
    public static double manp = .0007, mani = 0, mand = 0.00007, manf = -.10;

    public enum Rotations {
        REST(200),
        SUB(1700),
        SUBGRAB(1300),
        WALL(1850),
        BAR(4400),
        TEST(5000);
        public int position;
        Rotations(int position) {this.position = position;}

        public int getPosition() {return this.position;}
    }
    //public Telemetry telemetry;
    private PIDController rotatePIDController, extensionPIDController, manualPIDController;

    public int rotationTarget = 0;
    private int extensionTarget = 0;
    public static int maxRotation = 9400;
    public static int minRotation = -100;
    public static int maxExtension = -50000;
    public static int minExtension = 8000;
    public static boolean usef = true;
    public static boolean useRotation = true;
    public static boolean useExtension = true;
    public boolean manual = false;




    public ArmWithPID(HardwareMap hardwaremap) {
        rotate1 = hardwaremap.get(DcMotorEx.class, "Arm Rotate 1");
        rotate2 = hardwaremap.get(DcMotorEx.class, "Arm Rotate 2");
        extend1 = hardwaremap.get(DcMotorEx.class, "Arm Extend 1");
        extend2 = hardwaremap.get(DcMotorEx.class, "Arm Extend 2");
        rotatePIDController = new PIDController(rotp, roti, rotd);
        extensionPIDController = new PIDController(extp, exti, extd);
        manualPIDController = new PIDController(manp, mani, mand);
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
        setExtensionTarget(getExtensionPosition());
        setRotateTarget(getRotationPosition());

    }

    public void update() {
        rotatePIDController.setPID(rotp, roti, rotd);
        extensionPIDController.setPID(extp, exti, extd);
        manualPIDController.setPID(manp,mani,mand);

        double rotPos = rotate2.getCurrentPosition();
        double extPos = getExtensionPosition();
        double rotPID;
        if (manual) {
            rotPID = manualPIDController.calculate(rotPos, rotationTarget);
        } else {
            rotPID = rotatePIDController.calculate(rotPos, rotationTarget);
        }
        double extPID = extensionPIDController.calculate(extPos, extensionTarget);

        //maybe no tick in degrees
        double rotff;
        double extff;
        if (usef) {
            rotff = Math.cos(Math.toRadians(rotationTarget / rot_ticks_in_degree)) * rotf;
            extff = Math.cos(Math.toRadians(extensionTarget / ext_ticks_in_degree)) * extf;
        } else {
            rotff = 0;
            extff = 0;
        }

        double rotPower = Range.clip(rotPID+rotff, -1, 1);
        double extPower = Range.clip(extPID+extff, -1,1);
//        telemetry.addData("ROT TARGET: ", rotPower);
//        telemetry.update();
//        telemetry.addData("Power: ", rotPower);
//        telemetry.update();
        if (useRotation) {
            setRotatePower(-rotPower);
        }
        if (useExtension) {
            setExtendPower(extPower);
        }
    }
    public boolean reachRotateTarget() {
        return Math.abs(rotate2.getVelocity()) <=100;
    }
    public void setRotateTarget(int target) {
        rotationTarget = Range.clip(target,minRotation, maxRotation);
    }

    public void rotateRest() {
        manual = false;
        rotationTarget = Rotations.REST.getPosition();
    }
    public void rotateTest() {
        manual = false;
        rotationTarget = Rotations.TEST.getPosition();
    }
    public void rotateSub() {
        manual = false;
        rotationTarget = Rotations.SUB.getPosition();
    }
    public void rotateGrabSub() {
        manual = false;
        rotationTarget = Rotations.SUBGRAB.getPosition();
    }

    public void rotateWall() {
        manual = false;
        rotationTarget = Rotations.WALL.getPosition();
    }
    public void rotateBar() {
        manual = false;
        rotationTarget = Rotations.BAR.getPosition();
    }

    public void rotateFull() {
        manual = false;
        rotationTarget = maxRotation;
    }

    public void setExtensionTarget(int target) {
        extensionTarget = Range.clip(target,maxExtension,minExtension);
    }

    public void fullExtend() {
        extensionTarget = maxExtension+100;
    }

    public void fullRetract() {
        extensionTarget = minExtension-100;
    }

    public void manualRotate(double power) {
        if (power > .3) {
            manual = true;
        }
        setRotateTarget(rotationTarget+(int) (power*300));
    }

    public void manualExtend(double power) {
        setExtensionTarget(extensionTarget+(int) (power*1000));
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
    public int getRotationTarget() {
        return rotationTarget;
    }
    public int getExtensionTarget() {
        return extensionTarget;
    }

    public double getRotationVelocity() {return rotate2.getVelocity();}
    public boolean isRetracted() {return getExtensionPosition()>=minExtension-3000;}

}
