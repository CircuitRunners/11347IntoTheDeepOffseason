package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Diffy;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.RollerIntake;

@TeleOp
public class TeleOpV2 extends CommandOpMode {
    public Drivebase drivebase;
    public Arm arm;
    //public RollerIntake intake;
    public Diffy diffy;

    @Override
    public void initialize() {
        drivebase = new Drivebase(hardwareMap);
        arm = new Arm(hardwareMap);
        //intake = new RollerIntake(hardwareMap);
        diffy = new Diffy(hardwareMap);
    }

    @Override
    public void run() {
        super.run();
        drivebase.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        arm.manualRotate(gamepad2.right_stick_y);
        arm.manualExtend(gamepad2.left_stick_y);
        if (gamepad2.right_bumper || gamepad2.left_bumper) {
            diffy.toggleClaw();
        }

        if (gamepad2.dpad_up) {
            diffy.angleDiffyManual(-1);
        } else if (gamepad2.dpad_down) {
            diffy.angleDiffyManual(1);
        }

        if (gamepad2.dpad_left) {
            diffy.rotateDiffyManual(-1);
        } else if (gamepad2.dpad_right) {
            diffy.rotateDiffyManual(1);
        }


    }
}
