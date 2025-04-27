package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Diffy;

@TeleOp
@Config
public class DiffyTest extends CommandOpMode {
    public Diffy diffy;
    //public Telemetry telemetry;
    public static double rightPos = 0.5;
    public static double leftPos = 0.5;
    public static boolean setPos = false;
    @Override
    public void initialize() {
        diffy = new Diffy(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        super.run();
        if (setPos) {
            diffy.setRightDiffyPosition(rightPos);
            diffy.setLeftDiffyPosition(leftPos);
        } else {
            diffy.moveleftServo(gamepad2.left_stick_y);
            diffy.moveRightServo(gamepad2.right_stick_y);

            if (gamepad2.dpad_left) {
                diffy.angleDiffyManual(-1);
            } else if (gamepad2.dpad_right) {
                diffy.angleDiffyManual(1);
            }
            if (gamepad2.dpad_up) {
                diffy.rotateDiffyManual(1);
            } else if (gamepad2.dpad_down) {
                diffy.rotateDiffyManual(-1);
            }
        }

        telemetry.addData("Left Position", diffy.leftServoPosition());
        telemetry.addData("Right Position", diffy.rightServoPosition());
        telemetry.update();
    }
}
