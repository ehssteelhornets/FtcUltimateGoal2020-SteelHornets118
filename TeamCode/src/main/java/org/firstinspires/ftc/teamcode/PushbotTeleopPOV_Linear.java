package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Pushbot: Teleop POV", group = "Pushbot")
public class PushbotTeleopPOV_Linear extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    Pushbot_2020 robot = new Pushbot_2020();
    boolean liftMoving = false;
    double liftPos;

    @Override
    public void runOpMode() {
        // Save reference to Hardware map
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        boolean precisionMode = false;
        telemetry.addData("opModeIsActive", opModeIsActive());
        telemetry.update();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetry.update();


            //Gamepad 1: Claw, Intake, Rev, Shoot, Drive, Precision Mode

            // Claw Controlled by dpad up and down
            if (gamepad1.dpad_down) {
                robot.openClaw(true);
            } else if (gamepad1.dpad_up) {
                robot.openClaw(false);
            }

            // Intake driven by bumpers
            if (gamepad1.left_bumper) {
                // Intake in
                robot.intake.setPower(.7);
            } else if (gamepad1.right_bumper) {
                // Intake out
                robot.intake.setPower(-.7);
            } else {
                robot.intake.setPower(0);
            }

            // Rev driven by left trigger
            robot.rev(gamepad1.left_trigger > 0);

            // Launch driven by right trigger
            robot.launch(gamepad1.right_trigger > .5);

            // Driving with
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            // Precision Mode
            if (precisionMode) {
                //telemetry.addData("Driving", "false");
                //telemetry.update();
                robot.driveLF.setPower(v1 / 4);
                robot.driveRF.setPower(v2 / 4);
                robot.driveLB.setPower(v3 / 4);
                robot.driveRB.setPower(v4 / 4);
            } else {
                // Normal Mode
                //telemetry.addData("Driving", "true");
                //telemetry.update();
                robot.driveLF.setPower(v1);
                robot.driveRF.setPower(v2);
                robot.driveLB.setPower(v3);
                robot.driveRB.setPower(v4);
            }

            // Precision Mode Toggle
            if (gamepad1.b) {
                precisionMode = false;
            } else if (gamepad1.x) {
                precisionMode = true;
            }


            // Gamepad 2: Arm, Lift, Barrel

            // Arm driven by bumpers
            if (gamepad2.left_bumper) {
                robot.armMotor.setPower(-.5);
            } else if (gamepad2.right_bumper) {
                robot.armMotor.setPower(.5);
            } else {
                robot.armMotor.setPower(0);
            }

            // Ring lift driven by dpad
            if (gamepad2.dpad_right) {
                robot.currLiftPos = 0;
                liftMoving = true;
                //robot.lift.setPower(-1);
                //robot.liftUp(1.0);
            } else if (gamepad2.dpad_down) {
                robot.currLiftPos = 1;
                liftMoving = true;
                //robot.lift.setPower(-1);
                //robot.liftDown(1.0);
            } else if (gamepad2.dpad_left) {
                robot.currLiftPos = 2;
                liftMoving = true;
                //robot.lift.setPower(-1);
                //robot.liftUp(1.0);
            } else if (gamepad2.dpad_up) {
                robot.currLiftPos = 3;
                liftMoving = true;
                //robot.lift.setPower(1);
                //robot.liftDown(1.0);
            }/*else {
                robot.lift.setPower(0);
            }*/
            if (liftMoving) {
                liftPos = robot.liftPos[robot.currLiftPos];
                if (robot.currLiftPos == 0) {
                    if (robot.getBetterDistance() < liftPos) {
                        robot.lift.setPower(-1 * robot.up);
                    } else {
                        liftMoving = false;
                        robot.lift.setPower(0);
                    }
                } else {
                    if (robot.getBetterDistance() > liftPos) {
                        robot.lift.setPower(1.0 * robot.up);
                    } else {
                        liftMoving = false;
                        robot.lift.setPower(0);
                    }
                }
            }


            // Barrel alignment driven by letter buttons
            if (gamepad2.x) {
                robot.barrel(3);
            } else if (gamepad2.y) {
                robot.barrel(2);
            } else if (gamepad2.b) {
                robot.barrel(1);
            } else if (gamepad2.a) {
                robot.barrel(0);
            }

            // Get range sensor data
            telemetry.addData("raw ultrasonic", robot.liftSensor.rawUltrasonic());
            telemetry.addData("raw optical", robot.liftSensor.rawOptical());
            telemetry.addData("liftPos:", "%2.2f cm", liftPos);
            telemetry.addData("Super cm:", "%.2f cm", robot.getBetterDistance());
            telemetry.update();
        }
    }
}
