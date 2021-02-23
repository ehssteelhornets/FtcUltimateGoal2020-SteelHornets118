package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@Autonomous(name = "Parking_Wobble_Auto", group = "Pushbot")
public class Parking_Wobble_Auto extends LinearOpMode {
    Pushbot_2020 robot = new Pushbot_2020();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    // eg: AndyMark Orbital 20 Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: AndyMark Orbital 20 Motor Encoder from Video
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP AndyMark Orbital 20
    static final double WHEEL_DIAMETER_INCHES = 3.75;     // For figuring circumference
    static final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415) / 1.01; //1.01 is a manual adjustment
    //TODO: Get new CPI for the motors
    float[] hsvValues = {0F, 0F, 0F};

    @Override
    public void runOpMode() {
        //Initializes the robot
        robot.init(hardwareMap);
        //Reset our encoders
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        robot.driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.driveLF.getCurrentPosition(),
                robot.driveRF.getCurrentPosition());
        telemetry.update();
        waitForStart();

        robot.openClaw(true);
        sleep(500);
        robot.armOut(true);
        sleep(500);
        robot.openClaw(false);
        sleep(500);
        robot.encoderDrive(.4, 108, 'F');
        robot.openClaw(true);
        sleep(500);
        robot.armOut(false);
        sleep(500);
        robot.encoderDrive(.4, 63, 'B');
        sleep(500);/*
        robot.autoFire('R');
        sleep(1000);*/
        robot.encoderDrive(.4, 18, 'F');
    }
/*
    public void encoderDrive(double speed,
                             double dist, // in inches
                             char dir ) {
        int TIMEOUT = 10;

        int target;
        // Ensure that the opmode is still active
        try {
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                target = (int) (dist * COUNTS_PER_INCH);
                // Decide which direction each motor should go
                if (dir == 'F') {
                    robot.driveLF.setTargetPosition(target);
                    robot.driveRF.setTargetPosition(target);
                    robot.driveLB.setTargetPosition(target);
                    robot.driveRB.setTargetPosition(target);
                } else if (dir == 'B') {
                    robot.driveLF.setTargetPosition(-target);
                    robot.driveRF.setTargetPosition(-target);
                    robot.driveLB.setTargetPosition(-target);
                    robot.driveRB.setTargetPosition(-target);
                } else if (dir == 'L') {
                    robot.driveLF.setTargetPosition(-target);
                    robot.driveRF.setTargetPosition(target);
                    robot.driveLB.setTargetPosition(target);
                    robot.driveRB.setTargetPosition(-target);
                } else if (dir == 'R') {
                    robot.driveLF.setTargetPosition(target);
                    robot.driveRF.setTargetPosition(-target);
                    robot.driveLB.setTargetPosition(-target);
                    robot.driveRB.setTargetPosition(target);
                }
                // Turn On RUN_TO_POSITION
                robot.driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                runtime.reset();
                robot.driveLF.setPower(Math.abs(speed));
                robot.driveRF.setPower(Math.abs(speed));
                robot.driveLB.setPower(Math.abs(speed));
                robot.driveRB.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (runtime.seconds() < TIMEOUT) &&
                        (robot.driveLF.isBusy() && robot.driveRF.isBusy() && robot.driveRB.isBusy() && robot.driveLB.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", target, target);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.driveLF.getCurrentPosition(),
                            robot.driveRF.getCurrentPosition());
                    telemetry.addData("Back wheels", target + "" + target);
                    telemetry.update();
                }

                // Stop all motion
                robot.driveLF.setPower(0);
                robot.driveRF.setPower(0);
                robot.driveLB.setPower(0);
                robot.driveRB.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        } catch (TargetPositionNotSetException e) {
            telemetry.addData("Mission Failed", "We'll get 'em next time: ");
            telemetry.update();
        }
    }

    public void encoderTurn(double speed,
                            double ang,
                            boolean cWise) {
        int TIMEOUT = 10;
        double COUNTS_PER_DEGREE = 11.85;

        int target;
        // Ensure that the opmode is still active
        try {
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                target = (int) (ang * COUNTS_PER_DEGREE);
                // Decide which direction each motor should go
                if (cWise) {
                    robot.driveLF.setTargetPosition(target);
                    robot.driveRF.setTargetPosition(-target);
                    robot.driveLB.setTargetPosition(target);
                    robot.driveRB.setTargetPosition(-target);
                } else {
                    robot.driveLF.setTargetPosition(-target);
                    robot.driveRF.setTargetPosition(target);
                    robot.driveLB.setTargetPosition(-target);
                    robot.driveRB.setTargetPosition(target);
                }
                // Turn On RUN_TO_POSITION
                robot.driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                runtime.reset();
                robot.driveLF.setPower(Math.abs(speed));
                robot.driveRF.setPower(Math.abs(speed));
                robot.driveLB.setPower(Math.abs(speed));
                robot.driveRB.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (runtime.seconds() < TIMEOUT) &&
                        (robot.driveLF.isBusy() && robot.driveRF.isBusy() && robot.driveRB.isBusy() && robot.driveLB.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", target, target);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.driveLF.getCurrentPosition(),
                            robot.driveRF.getCurrentPosition());
                    telemetry.addData("Back wheels", target + "" + target);
                    telemetry.update();
                }

                // Stop all motion
                robot.driveLF.setPower(0);
                robot.driveRF.setPower(0);
                robot.driveLB.setPower(0);
                robot.driveRB.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        } catch (TargetPositionNotSetException e) {
            telemetry.addData("Mission Failed", "We'll get 'em next time: ");
            telemetry.update();
        }
    }*/
}
