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

@TeleOp
public class TeleOpTwoGamepad extends CommandOpMode {
    public FieldCentricDrive drivebase;
    public ArmWithPID arm;
    //public RollerIntake intake;
    public Diffy diffy;
    //public Telemetry telemetry;
    GamepadEx driver, manipulator;
    public boolean inSub = false;
    @Override
    public void initialize() {
        //schedule(new BulkCacheCommand(hardwareMap));
        drivebase = new FieldCentricDrive(hardwareMap);
        arm = new ArmWithPID(hardwareMap);
        //intake = new RollerIntake(hardwareMap);
        diffy = new Diffy(hardwareMap);
        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Initialized");
        telemetry.update();

        //Automations
            //Extend to sub
            new Trigger(() -> manipulator.getButton(GamepadKeys.Button.DPAD_LEFT) && !inSub).whenActive(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                diffy.rotateDiffyCenter();
                                arm.rotateSub();
                            }),
                            new WaitCommand(500),
                            new InstantCommand(() -> {
                                arm.fullExtend();
                            }),
                            new WaitCommand(1000),
                            new InstantCommand(() -> {
                                diffy.rotateDiffyDown();
                                diffy.openClaw();
                                inSub = true;
                            })
                    ));
            //Grab from sub
            new Trigger(() -> manipulator.getButton(GamepadKeys.Button.DPAD_LEFT) && inSub).whenActive(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                arm.rotateGrabSub();
                            }),
                            new WaitCommand(250),
                            new InstantCommand(() -> {
                                diffy.closeClaw();
                            }),
                            new WaitCommand(250),
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
            manipulator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                arm.fullRetract();
                                diffy.closeClaw();
                            }),
                            new WaitCommand(250),
                            new InstantCommand(() -> {
                                diffy.rotateDiffyUp();
                                if (arm.isRetracted()) {
                                    arm.rotateRest();
                                }
                            })
                    ));
            //Grab from wall
        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            arm.rotateWall();
                            diffy.openClaw();
                            diffy.rotateDiffyCenter();
                        })
                ));
        //High Bar
        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            //Need to test
                            diffy.closeClaw();
                            sleep(250);
                            //
                            arm.rotateFull();
                            diffy.rotateDiffyUp();
                        })
                ));
        //High bucket
        manipulator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            arm.rotateFull();
                        }),
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            arm.fullExtend();
                            diffy.rotateDiffyCenter();
                        })
                ));

            //Tests
        /*
        manipulator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            arm.rotateTest();
                        }),
                        //new WaitCommand(500),
                        new InstantCommand(() -> {
                            if (arm.reachRotateTarget()) {
                                arm.rotateRest();
                            }
                        })
                ));*/
    }

    @Override
    public void run() {
        super.run();
        //gamepad 1
            drivebase.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.square) {
                drivebase.resetIMU();
            }
            if (gamepad1.right_trigger>.3) {
                drivebase.setPowerMultiplier(.4);
            } else {
                drivebase.setPowerMultiplier(.8);
            }

        //gamepad 2

            //Arm

                arm.manualRotate(-gamepad2.right_stick_y);
                arm.manualExtend(gamepad2.left_stick_y);
                arm.update();

            //Claw
                if (gamepad1.right_bumper || gamepad1.left_bumper) {
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
