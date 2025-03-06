package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.ArmWithPID;
import org.firstinspires.ftc.teamcode.Subsystems.Diffy;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.RollerIntake;
@Config
@TeleOp
public class ArmTester extends CommandOpMode {
    public ArmWithPID arm;
    //public RollerIntake intake;
    public static int rotateTarget;
    public static int extendTarget;
    public static boolean shouldRotate = true;
    public static boolean shouldExtend = true;
    //public Telemetry telemetry;

    @Override
    public void initialize() {

        arm = new ArmWithPID(hardwareMap);
        //intake = new RollerIntake(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        if (shouldRotate) {
            arm.setRotateTarget(rotateTarget);
        }
        if (shouldExtend) {
            arm.setExtensionTarget(extendTarget);
        }
        arm.update();


        telemetry.addData("Arm Rotation", arm.getRotationPosition());
        telemetry.addData("Rotation Target", rotateTarget);
        telemetry.addData("Arm Extension", arm.getExtensionPosition());
        telemetry.addData("Extension Target", extendTarget);

        telemetry.update();

    }
}
