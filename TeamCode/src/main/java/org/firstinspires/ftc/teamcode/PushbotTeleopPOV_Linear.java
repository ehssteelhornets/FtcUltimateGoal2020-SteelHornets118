package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Pushbot: Teleop POV", group = "Pushbot")
public class PushbotTeleopPOV_Linear extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    Pushbot_2020 robot = new Pushbot_2020();
    int liftPos = 1;

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
            telemetry.update();

            // Select mode
            if (gamepad1.right_stick_button) {
                precisionMode = !precisionMode;
                sleep(100);
            }

            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            // Precision Mode
            if (precisionMode) {
                telemetry.addData("Driving", "false");
                telemetry.update();
                robot.driveLF.setPower(v1/4);
                robot.driveRF.setPower(v2/4);
                robot.driveLB.setPower(v3/4);
                robot.driveRB.setPower(v4/4);
            } else {
                // Normal Mode
                telemetry.addData("Driving", "true");
                telemetry.update();
                robot.driveLF.setPower(v1);
                robot.driveRF.setPower(v2);
                robot.driveLB.setPower(v3);
                robot.driveRB.setPower(v4);
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

            // Arm driven by dpad left and right
            if (gamepad1.dpad_left) {
                robot.armMotor.setPower(.5);
            } else if (gamepad1.dpad_right) {
                robot.armMotor.setPower(-.5);
            } else {
                robot.armMotor.setPower(0);
            }

            // Claw Controlled by dpad up and down
            if (gamepad1.dpad_down) {
                robot.openClaw(true);
            } else if (gamepad1.dpad_up) {
                robot.openClaw(false);
            }

            // Rev driven by left trigger
            robot.rev(gamepad1.left_trigger > 0);

            // Launch driven by right trigger
            robot.launch(gamepad1.right_trigger > 0);

            // Ring lift driven by a and y
            if (gamepad1.a) {
                robot.lift.setPower(1);
                //robot.liftDown();
            } else if (gamepad1.y) {
                robot.lift.setPower(-1);
                //robot.liftUp();
            } else {
                robot.lift.setPower(0);
            }

            // Barrel alignment driven by x and b
            if (gamepad1.b) {
                robot.barrel(2);
            } else if (gamepad1.x) {
                if (precisionMode) {
                    robot.barrel(1);
                } else {
                    robot.barrel(0);
                }
            }

            // Get range sensor data
            telemetry.addData("raw ultrasonic", robot.liftSensor.rawUltrasonic());
            telemetry.addData("raw optical", robot.liftSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", robot.liftSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", robot.liftSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
