package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmWithPID;

@Config
@TeleOp
public class ArmTester extends CommandOpMode {
    public ArmWithPID arm;
    //public RollerIntake intake;
    public static int rotateTarget;
    public static int extendTarget;
//    public static boolean shouldRotate = true;
//    public static boolean shouldExtend = true;
    public static boolean shouldUpdate = true;
    //public Telemetry telemetry;
    public static int pastPosition = 0;

    @Override
    public void initialize() {

        arm = new ArmWithPID(hardwareMap);
        //intake = new RollerIntake(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
        pastPosition = arm.getRotationPosition();
    }

    @Override
    public void run() {
        pastPosition = arm.getRotationPosition();
        super.run();
            arm.setRotateTarget(rotateTarget);
            arm.setExtensionTarget(extendTarget);
        if (shouldUpdate) {
            arm.update();
        }


        telemetry.addData("Arm Rotation", arm.getRotationPosition());
        telemetry.addData("Rotation Target", rotateTarget);
        telemetry.addData("Arm Extension", arm.getExtensionPosition());
        telemetry.addData("Extension Target", extendTarget);
        telemetry.addData("Rotation position difference", arm.getRotationPosition()-pastPosition);
        telemetry.update();

    }
}
