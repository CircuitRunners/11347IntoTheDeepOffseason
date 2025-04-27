package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmWithPID;
import org.firstinspires.ftc.teamcode.Subsystems.Diffy;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
/*Button mapping:
Drivebase:
    Left Stick: drivebase drive
    Left stick button: reset IMU
    Right stick: Rotate drivebase
Diffy:
    Left D-pad: Rotate diffy left
    Right D-pad: Rotate diffy right
    Up D-pad: Rotate diffy up
    Down D-pad: Rotate diffy down
    X (Bottom button): Toggle claw
Arm:
    Right Trigger: Rotate arm up
    Left Trigger: Rotate arm down
    Right Bumper: Extend Arm
    Left Bumper: Retract Arm
Automations:
    Square (Left Button): Extend to submersible if not there, grab if there
    Triangle (Top Button): Retract from submersible
    Circle (Right Button): Specimen cycle (Wall/High Bar)
    Touchpad: Reset Mechanisms
 */

@TeleOp
public class TeleOpSpecOneGamepad extends CommandOpMode {
    public FieldCentricDrive drivebase;
    public ArmWithPID arm;
    //public RollerIntake intake;
    public Diffy diffy;
    //public Telemetry telemetry;
    GamepadEx gamepad;
    public boolean inSub = false;
    public boolean specStage = false; //True if at wall
    @Override
    public void initialize() {
        //schedule(new BulkCacheCommand(hardwareMap));
        drivebase = new FieldCentricDrive(hardwareMap);
        arm = new ArmWithPID(hardwareMap);
        //intake = new RollerIntake(hardwareMap);
        diffy = new Diffy(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Initialized");
        telemetry.update();

        //Automations
        //Extend to sub
        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.X) && !inSub).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            diffy.rotateDiffyCenter();
                            arm.rotateSub();
                        }),
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            arm.fullExtend();
                        }),
                        new WaitCommand(1100),
                        new InstantCommand(() -> {
                            diffy.rotateDiffyDown();
                            diffy.openClaw();
                            inSub = true;
                        })
                ));
        //Grab
        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.X) && inSub).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            diffy.openClaw();
                            arm.rotateGrabSub();
                        }),
                        new WaitCommand(250),
                        new InstantCommand(() -> {
                            diffy.closeClaw();
                        }),
                        new WaitCommand(250),
                        new InstantCommand(() -> {
                            arm.rotateSub();
                        })
                ));
        //Retract from sub
        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.Y) && inSub).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            arm.rotateSub();
                            diffy.rotateDiffyCenter();
                        }),
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            arm.fullRetract();
                            inSub = false;
                        })/*
                            new WaitCommand(1250),
                            new InstantCommand(() -> {
                                arm.rotateRest();
                            })*/
                ));

        //Reset
        new Trigger(() -> gamepad1.touchpad).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            arm.fullRetract();
                            diffy.closeClaw();
                            diffy.rotateDiffyUp();
                            if (arm.isRetracted()) {
                                arm.rotateRest();
                            }
                        })/*
                        new WaitCommand(250),
                        new InstantCommand(() -> {
                            diffy.rotateDiffyUp();
                            if (arm.isRetracted()) {
                                arm.rotateRest();
                            }
                        })*/
                ));

        //Grab from wall
        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.B) && !specStage).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            arm.rotateWall();
                            diffy.openClaw();
                            diffy.rotateDiffyCenter();
                            specStage = true;
                        })
                ));

        //High Bar
        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.B) && specStage).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            //Need to test
                            diffy.closeClaw();
                            sleep(250);
                            arm.rotateFull();
                            diffy.rotateDiffyUp();
                            specStage = false;
                        })
                ));
    }

    @Override
    public void run() {
        super.run();
        //Drivebase
        drivebase.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if (gamepad1.right_stick_button) {
            drivebase.resetIMU();
        }/*
        if (gamepad1.right_trigger>.3) {
            drivebase.setPowerMultiplier(.4);
        } else {
            drivebase.setPowerMultiplier(.8);
        }*/

        //Arm

        arm.manualRotate(-(gamepad1.right_trigger-gamepad1.left_trigger));
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            arm.manualExtend(1);
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper){
            arm.manualExtend(-1);
        }
        arm.update();

        //Claw
        if (gamepad1.a) {
            diffy.toggleClaw();
        }

        //Diffy
        if (gamepad1.dpad_left) {
            diffy.angleDiffyManual(-1);
        } else if (gamepad1.dpad_right) {
            diffy.angleDiffyManual(1);
        }
        if (gamepad1.dpad_up) {
            diffy.rotateDiffyManual(1);
        } else if (gamepad1.dpad_down) {
            diffy.rotateDiffyManual(-1);
        }

        telemetry.addData("Arm Rotation",arm.getRotationPosition());
        telemetry.addData("Rotation Target",arm.getRotationTarget());
        telemetry.addData("Rotation Speed",arm.getRotationVelocity());
        telemetry.addData("Arm Extension",arm.getExtensionPosition());
        telemetry.addData("Extension Target", arm.getExtensionTarget());
        telemetry.addLine("Is Claw Open? " + diffy.isClawOpen());
        telemetry.addLine("Claw Timer: " + diffy.switchTimer);
        telemetry.addLine("Left Diffy Servo Position: " + diffy.leftServoPosition());
        telemetry.addLine("Right Diffy Servo Position: " + diffy.rightServoPosition());
        telemetry.update();

    }
}
