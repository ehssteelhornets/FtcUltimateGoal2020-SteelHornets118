package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
public class PushbotTeleopPOV_Linear extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static double right;
    private static double left;
    private static double right2;
    private static double left2;
    Pushbot_2020 robot = new Pushbot_2020();

    @Override
    public void runOpMode() {
        // Save reference to Hardware map
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        int mode = 1;
        telemetry.addData("opModeIsActive", opModeIsActive());
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.update();
            if (gamepad1.right_stick_button && mode == 1) {
                mode++;
                sleep(100);
            } else if (gamepad1.right_stick_button && mode == 2) {
                mode--;
                sleep(100);
            }
            //Tank Drive
            if (mode == 2) {
                telemetry.addData("Driving", "false");
                telemetry.update();
                right = gamepad1.right_stick_y;
                left = gamepad1.left_stick_y;
                boolean precisionMode = false;
                if (gamepad1.left_trigger != 0)
                    precisionMode = true;
                boolean reverseMode = false;
                if (gamepad1.right_trigger != 0)
                    reverseMode = true;
                drive(precisionMode, reverseMode);
            } else if (mode == 1) {
                //Mech Drive
                telemetry.addData("Driving", "true");
                telemetry.update();
                double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;
                robot.driveLF.setPower(v1);
                robot.driveRF.setPower(v2);
                robot.driveLB.setPower(v3);
                robot.driveRB.setPower(v4);
            }

            // Intake driven by right trigger
            if(gamepad1.right_trigger > 0) {
                robot.intake.setPower(1);
            } else {
                robot.intake.setPower(0);
            }

            if(gamepad1.a) {
                robot.armOut(true);
            } else if(gamepad1.b) {
                robot.armOut(false);
            }

            // Claw Controlled by dpad down and
            if(gamepad1.dpad_down) {
                robot.openClaw(true);
            } else if(gamepad1.dpad_left) {
                robot.openClaw(false);
            }

            /*
            //Continuous rotation of CRServo
            if (gamepad1.a) {
                robot.testServo.setPower(0.8);
            }
            if (gamepad1.b) {
                robot.testServo.setPower(-0.8);
            }
            telemetry.update();
             */

            //foundation hook toggle
            /*if (gamepad2.a && foundHook != 0) {
                foundHook--;
                sleep(300);
            } else if (gamepad2.a) {
                foundHook++;
                sleep(300);
            }

            if (foundHook != 0) {
                robot.foundHook1.setPosition(1.0);
                robot.foundHook2.setPosition(0.0);
            }
            telemetry.addData("Raising foundation hooks", "true");
            telemetry.update();

            if (foundHook == 0) {
                robot.foundHook1.setPosition(0.0);
                robot.foundHook2.setPosition(1.0);
            }
            telemetry.addData("Lowering foundation hooks", "true");
            telemetry.update();

            //Controls linear actuator
            // Show the elapsed game time and wheel powerhhy
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", right, left);
            telemetry.update();

            //linear actuator
            robot.vertExt.setPower(gamepad2.left_stick_y);
            telemetry.addData("Moving the linear actuator", "true");
            telemetry.update();*/

            /* old linear actuator code
            if (gamepad2.dpad_up) {
                robot.vertExt.setPower(-1);
            } else if (gamepad2.dpad_down) {
                robot.vertExt.setPower(1);
            } else if (gamepad2.dpad_right) {
                robot.vertExt.setPower(0);
            */

            //tape extender
            /*robot.tapeExt.setPower(gamepad2.right_stick_y);
            telemetry.addData("Moving the tape extender", "true");
            telemetry.update();*/


            // secondstone grabber mech
            /*
            if (gamepad1.dpad_up) {
                robot.stoneClaw.setPower(-0.05);
            } else if (gamepad1.dpad_down) {
                robot.stoneClaw.setPower(0);
            } else if (gamepad1.dpad_right) {
                robot.stoneClaw.setPower(-0.001);
            } else if (gamepad1.dpad_left) {
                robot.stoneClaw.setPower(-.05);
                sleep(700);
                robot.stoneClaw.setPower(-.001);
            }
            */
            //old Stone grabber Mech

            /*if (gamepad1.right_bumper) {//Open complete
                //robot.rightClaw.setPosition(0.65);
                robot.rightClaw.setPosition(0.8);
                //robot.leftClaw.setPosition(0.2); old
                robot.leftClaw.setPosition(0.0);
                telemetry.addData("Closing grabber", "true");
                telemetry.update();
            }
            else if (gamepad1.left_bumper) {//Opened
                //clawOffset = 0.0;
                //inversely proportional
                robot.rightClaw.setPosition(0.5);//WORKS PERFECTLY SRIJAN (GOOD JOB)
                robot.leftClaw.setPosition(0.35);
                telemetry.addData("Opening grabber moderate amount", "true");
                telemetry.update();
            }
            else if (gamepad1.left_trigger != 0) {//CLOSED
            //clawOffset = 0.0;
            //inversely proportional
                robot.rightClaw.setPosition(0.2);//WORKS PERFECTLY SRIJAN (GOOD JOB)
                robot.leftClaw.setPosition(0.65);
                telemetry.addData("Opening grabber large amount", "true");
                telemetry.update();
            }*/

            //tuckawayclaw toggle
            /*if (gamepad2.x && tuckAway != 0) {
                tuckAway--;
                sleep(300);
            } else if (gamepad2.x) {
                tuckAway++;
                sleep(300);
            }

            if (tuckAway == 0) {
                robot.tuckAwayClaw1.setPower(.8);
                robot.tuckAwayClaw2.setPower(-.4);
            }
            telemetry.addData("moving tuckAway claw out", "true");
            telemetry.update();

            if (tuckAway != 0) {
                robot.tuckAwayClaw1.setPower(-.5);
                robot.tuckAwayClaw2.setPower(0.9);
            }
            telemetry.addData("moving tuckAway claw in", "true");
            telemetry.update();*/
        }
    }
    void drive (boolean precise, boolean reverse){
        double right_scaled = scaleMotor(right, precise);
        double left_scaled = scaleMotor(left, precise);

        if (reverse) {
            double temp = right_scaled;
            right_scaled = -left_scaled;
            left_scaled = -temp;
        }
        //Set power for motors
        robot.driveRF.setPower(right_scaled);
        robot.driveRB.setPower(right_scaled);
        robot.driveLF.setPower(left_scaled);
        robot.driveLB.setPower(left_scaled);
    }
    double scaleMotor ( double num, boolean precise){
        if (num == 0.0)
            return 0.0;
        //For precision mode
        double[] scaleArray = {0.5, 0.75, 1.0};
        double[] preciseArray = {0.1, 0.2, 0.3};
        // get the corresponding index for the scaleInput array.
        int index = (int) (num * (scaleArray.length - 1));
        index = Math.abs(index);
        double scaled;
        if (precise)
            scaled = preciseArray[index];
        else
            scaled = scaleArray[index];
        if (num < 0.0)
            scaled = -scaled;
        return scaled;
    }
}
