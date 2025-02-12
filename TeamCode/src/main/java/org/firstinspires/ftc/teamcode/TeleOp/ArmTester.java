package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    public int rotateTarget;
    public int extendTarget;

    @Override
    public void initialize() {

        arm = new ArmWithPID(hardwareMap);
        //intake = new RollerIntake(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        arm.setRotateTarget(rotateTarget);
        arm.setExtensionTarget(extendTarget);
        arm.update();


        telemetry.addLine("Arm Rotation: " + arm.getRotationPosition());
        telemetry.addLine("Arm Extension: " + arm.getExtensionPosition());

    }
}
