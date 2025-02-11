package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Diffy {
    private Servo leftRotateServo, rightRotateServo, clawServo;
    private boolean isClawOpen;

    public enum ServoPosition {
        UP(0,0),//Need values
        DOWN(0,0),
        CENTER(0,0);
        private double leftServoPosition, rightServoPosition;

        ServoPosition(double leftServoPosition, double rightServoPosition) {
            this.leftServoPosition = leftServoPosition;
            this.rightServoPosition = rightServoPosition;
        }
    }
    private final ElapsedTime switchTimer = new ElapsedTime();

    public Diffy(HardwareMap hardwaremap) {
        leftRotateServo = hardwaremap.get(Servo.class, "Left Diffy Servo");
        rightRotateServo = hardwaremap.get(Servo.class, "Right Diffy Servo");
        clawServo = hardwaremap.get(Servo.class, "Claw Servo");
        clawServo.close();
        switchTimer.reset();

    }

    public void angleDiffyManual(double direction) {//should actually rotate
        leftRotateServo.setPosition(leftRotateServo.getPosition()+.005*direction);
        rightRotateServo.setPosition(rightRotateServo.getPosition()+.005*direction);

    }

    public void setDiffyAngle(double position) {
        leftRotateServo.setPosition(position);
        rightRotateServo.setPosition(position);
    }

    public void rotateDiffyManual(double direction) {//should actually angle
        leftRotateServo.setPosition(leftRotateServo.getPosition()+.005*direction);
        rightRotateServo.setPosition(rightRotateServo.getPosition()-.005*direction);

    }

    public void rotateDiffyUp() {
        leftRotateServo.setPosition(ServoPosition.UP.leftServoPosition);
        rightRotateServo.setPosition(ServoPosition.UP.rightServoPosition);
    }
    public void rotateDiffyCenter() {
        leftRotateServo.setPosition(ServoPosition.CENTER.leftServoPosition);
        rightRotateServo.setPosition(ServoPosition.CENTER.rightServoPosition);
    }
    public void rotateDiffyDown() {
        leftRotateServo.setPosition(ServoPosition.DOWN.leftServoPosition);
        rightRotateServo.setPosition(ServoPosition.DOWN.rightServoPosition);
    }

    public void setDiffyRotation(double position) {//Defintely going to need to change this
        leftRotateServo.setPosition(position);
        rightRotateServo.setPosition(-position);
    }

    public void openClaw() {
        clawServo.setPosition(0);
        isClawOpen = true;
    }

    public void closeClaw() {
        clawServo.setPosition(.19);
        isClawOpen = false;
    }

    public void toggleClaw() {
        if (switchTimer.milliseconds() > 500) {
            if (!isClawOpen) {
                openClaw();
            } else {
                closeClaw();
            }
        }
        switchTimer.reset();
    }

    public boolean isClawOpen() {
        return isClawOpen;
    }

    public double leftServoPosition() {
        return leftRotateServo.getPosition();
    }

    public double rightServoPosition() {
        return rightRotateServo.getPosition();
    }
}
