package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class Diffy {
    private Servo leftRotateServo, rightRotateServo, clawServo;
    private boolean isClawOpen;
    public static double rightSlow = .68;
    public static double manualSpeed = 0.02;

    public enum ServoPosition {
        UP(1,.5),//Need values
        DOWN(0,1),
        CENTER(.65,.65);
        private double leftServoPosition, rightServoPosition;

        ServoPosition(double leftServoPosition, double rightServoPosition) {
            this.leftServoPosition = leftServoPosition;
            this.rightServoPosition = rightServoPosition;
        }
    }
    public final ElapsedTime switchTimer = new ElapsedTime();

    public Diffy(HardwareMap hardwaremap) {
        leftRotateServo = hardwaremap.get(Servo.class, "Left Diffy Servo");
        rightRotateServo = hardwaremap.get(Servo.class, "Right Diffy Servo");
        clawServo = hardwaremap.get(Servo.class, "Claw Servo");
        clawServo.close();
        switchTimer.reset();

    }

    public void angleDiffyManual(double direction) {//should actually rotate
        leftRotateServo.setPosition(leftRotateServo.getPosition()+manualSpeed*direction);
        rightRotateServo.setPosition(rightRotateServo.getPosition()+manualSpeed*direction*rightSlow);

    }

    public void setDiffyAngle(double position) {
        leftRotateServo.setPosition(position);
        rightRotateServo.setPosition(position);
    }
    public void setRightDiffyPosition(double position) {
        rightRotateServo.setPosition(position);
    }
    public void setLeftDiffyPosition(double position) {
        leftRotateServo.setPosition(position);
    }

    public void rotateDiffyManual(double direction) {//should actually angle
        leftRotateServo.setPosition(leftRotateServo.getPosition()+manualSpeed*direction);
        rightRotateServo.setPosition(rightRotateServo.getPosition()-manualSpeed*direction*rightSlow);

    }

    public void moveRightServo(double direction) {
        rightRotateServo.setPosition(rightRotateServo.getPosition()+manualSpeed*direction*rightSlow);
    }
    public void moveleftServo(double direction) {
        leftRotateServo.setPosition(leftRotateServo.getPosition()+manualSpeed*direction);
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
            switchTimer.reset();
        }
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
