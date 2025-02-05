package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Diffy {
    private Servo leftRotateServo, rightRotateServo, clawServo;

    public Diffy(HardwareMap hardwaremap) {
        leftRotateServo = hardwaremap.get(Servo.class, "Left Diffy Servo");
        rightRotateServo = hardwaremap.get(Servo.class, "Right Diffy Servo");
        clawServo = hardwaremap.get(Servo.class, "Claw Servo");

    }

    public void angleDiffyManual(double direction) {
        leftRotateServo.setPosition(leftRotateServo.getPosition()+.1*direction);
        rightRotateServo.setPosition(rightRotateServo.getPosition()+.1*direction);

    }

    public void setDiffyAngle(double position) {
        leftRotateServo.setPosition(position);
        rightRotateServo.setPosition(position);
    }

    public void rotateDiffyManual(double direction) {
        leftRotateServo.setPosition(leftRotateServo.getPosition()+.1*direction);
        rightRotateServo.setPosition(rightRotateServo.getPosition()-.1*direction);

    }

    public void setDiffyRotation(double position) {//Defintely going to need to change this
        leftRotateServo.setPosition(position);
        rightRotateServo.setPosition(-position);
    }

    public void openClaw() {
        clawServo.setPosition(0);
    }

    public void closeClaw() {
        clawServo.setPosition(.19);
    }

    public void toggleClaw() {
        if (clawServo.getPosition()>.095) {
            openClaw();
        } else {
            closeClaw();
        }
    }
}
