package org.firstinspires.ftc.teamcode.oldCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RollerIntake {
    private CRServo rollerRight, rollerLeft;
    private Servo pivotServo, lock;

    public double unlockPosition = 80/360.0;
    public double lockPosition = 190/360.0;
    public double maxPivot = 300/360.0;//some values might be wrong
    public double midPivot = 127.5/360;
    public double minPivot = 75/360.0;

    public RollerIntake(HardwareMap hardwaremap) {
        rollerRight = hardwaremap.get(CRServo.class, "Right Roller");
        rollerLeft = hardwaremap.get(CRServo.class, "Left Roller");
        pivotServo = hardwaremap.get(Servo.class, "Pivot Servo");
        lock = hardwaremap.get(Servo.class, "Lock Servo");

    }

    public void intake(double power) {
        rollerRight.setPower(power);
        rollerLeft.setPower(power);
    }

    public void manualPivot(double direction) {//direction sign decides direction
        double newPosition = pivotServo.getPosition()+.1*direction;
        if (newPosition>=(maxPivot-.1)) {
            pivotServo.setPosition(maxPivot);
        } else if (newPosition<=(minPivot+.1)) {
            pivotServo.setPosition(minPivot);
        } else {
            pivotServo.setPosition(newPosition);
        }
    }

    public void setPivotPosition(double newPosition) {
        pivotServo.setPosition(newPosition);
    }

    public void unlock() {
        lock.setPosition(unlockPosition);
    }

    public void lock() {
        lock.setPosition(lockPosition);
    }

    public void toggleLock() {
        if (lock.getPosition()>.375) {
            unlock();
        } else {
            lock();
        }
    }



}
