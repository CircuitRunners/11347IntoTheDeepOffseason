package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;

@TeleOp
public class TestTeleOp extends CommandOpMode {
    public Drivebase drivebase;
    public Arm arm;
    //public Telemetry telemetry;

    @Override
    public void initialize() {
        drivebase = new Drivebase(hardwareMap);
        arm = new Arm(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        super.run();
        drivebase.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        arm.manualRotate(gamepad2.right_stick_y);
        arm.manualExtend(gamepad2.left_stick_y);
        telemetry.addData("Extend Position", arm.getExtensionPosition());
        telemetry.addData("Rotation Position", arm.getRotationPosition());
        telemetry.update();
    }
}
