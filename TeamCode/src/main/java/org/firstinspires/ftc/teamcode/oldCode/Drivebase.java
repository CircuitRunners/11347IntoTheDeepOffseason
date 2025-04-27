package org.firstinspires.ftc.teamcode.oldCode;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivebase extends SubsystemBase {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private double powerMultiplier = .8;
    //Need IMU
    public Drivebase(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");

        //often good. might not need to or with different motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double x, double y, double rx) {
        y = -y;
        x = x * 1.1;

        // Calculate the robot's heading from the IMU which not implemented yet
        //double botHeading = -getCorrectedYaw();

//        // botHeading is in Radians
//        Vector2d botVector = new Vector2d(x, y).rotated(botHeading);
//
////        // Apply the calculated heading to the input vector for field centric
//        x = botVector.getX(); // strafe r/l transform values
//        y = botVector.getY(); // strafe f/b transform values
//        // note rx is not here since rotation is always field centric!

        // Calculate the motor powers
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;

        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;

        // Find the max power and make sure it ain't greater than 1
        double denominator = max(
                max(abs(frontLeftPower), abs(backLeftPower)),
                max(abs(frontRightPower), abs(backRightPower))
        );

        if (denominator > 1.0) {
            frontLeftPower /= denominator;
            backLeftPower /= denominator;
            frontRightPower /= denominator;
            backRightPower /= denominator;
        }

        // Old method below, to ensure the CONTROLLERS no more than 1
        // double denominator = max(abs(y) + abs(x) + abs(rx), 1);

        // Set the motor powers
        leftFront.setPower(frontLeftPower*powerMultiplier);
        leftBack.setPower(backLeftPower*powerMultiplier);
        rightFront.setPower(frontRightPower*powerMultiplier);
        rightBack.setPower(backRightPower*powerMultiplier);
    }

}
