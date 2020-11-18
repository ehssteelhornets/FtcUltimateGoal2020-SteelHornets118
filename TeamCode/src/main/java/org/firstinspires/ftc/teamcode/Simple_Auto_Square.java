package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
@Autonomous(name="Simple_Auto_Square", group ="Pushbot")
public class Simple_Auto_Square extends LinearOpMode {
    Pushbot_2020 robot = new Pushbot_2020();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    // eg: AndyMark Orbital 20 Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Orbital 20 Motor Encoder from Video
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP AndyMark Orbital 20
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int test = 0;
    static final double DIST_TO_FOUNDATION = 30.75;

    @Override
    public void runOpMode() {
        //Initializes the robot
        robot.init(hardwareMap);
        //Reset our encoders
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();
        waitForStart();
        //Drives forward to foundation
        encoderDrive(1, -10, -10, 3);
        sleep(2000);
        rightStrafe(1, 10,3);
        sleep(2000);
        encoderDrive(1, 10, 10, 2);
        sleep(2000);
        leftStrafe(1,10,3);
        sleep(2000);
    }
//Encoder method
// Note: Reverse movement is obtained by setting a negative distance (not speed)

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;
        // Ensure that the opmode is still active
        try {
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                robot.leftDrive.setTargetPosition(newLeftTarget);
                robot.rightDrive.setTargetPosition(newRightTarget);
                robot.leftDrive2.setTargetPosition(newLeftTarget2);
                robot.rightDrive2.setTargetPosition(newRightTarget2);
                // Turn On RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftDrive.setPower(Math.abs(speed));
                robot.rightDrive.setPower(Math.abs(speed));
                robot.leftDrive2.setPower(Math.abs(speed));
                robot.rightDrive2.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftDrive.getCurrentPosition(),
                            robot.rightDrive.getCurrentPosition());
                    telemetry.addData("Back wheels", newLeftTarget2 + "" + newRightTarget2);
                    telemetry.update();
                }

                // Stop all motion
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftDrive2.setPower(0);
                robot.rightDrive2.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        } catch (TargetPositionNotSetException e) {
            telemetry.addData("Mission Failed", "We'll get 'em next time: " + test);
            telemetry.update();
        }
    }

    public void leftStrafe(double speed,
                           double Inches,
                           double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;
        // Ensure that the opmode is still active
        try {
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                robot.leftDrive.setTargetPosition(-newLeftTarget);
                robot.rightDrive.setTargetPosition(newRightTarget);
                robot.leftDrive2.setTargetPosition(newLeftTarget2);
                robot.rightDrive2.setTargetPosition(-newRightTarget2);
                // Turn On RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                runtime.reset();
                //robot.leftDrive.setPower(-Math.abs(speed));
                robot.leftDrive.setPower(Math.abs(speed));
                robot.rightDrive.setPower(Math.abs(speed));
                robot.leftDrive2.setPower(Math.abs(speed));
                //robot.rightDrive2.setPower(-Math.abs(speed));
                robot.rightDrive2.setPower(Math.abs(speed));

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftDrive.getCurrentPosition(),
                            robot.rightDrive.getCurrentPosition());
                    telemetry.addData("Back wheels", newLeftTarget2 + "" + newRightTarget2);
                    telemetry.update();
                }

                // Stop all motion
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftDrive2.setPower(0);
                robot.rightDrive2.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        } catch (TargetPositionNotSetException e) {
            telemetry.addData("Mission Failed", "We'll get 'em next time: " + test);
            telemetry.update();
        }
    }

    public void rightStrafe(double speed,
                            double Inches,
                            double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;
        // Ensure that the opmode is still active
        try {
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                robot.leftDrive.setTargetPosition(newLeftTarget);
                robot.rightDrive.setTargetPosition(-newRightTarget);
                robot.leftDrive2.setTargetPosition(-newLeftTarget2);
                robot.rightDrive2.setTargetPosition(newRightTarget2);
                // Turn On RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftDrive.setPower(Math.abs(speed));
                //robot.rightDrive.setPower(-Math.abs(speed));
                robot.rightDrive.setPower(Math.abs(speed));
                //robot.leftDrive2.setPower(-Math.abs(speed));
                robot.leftDrive2.setPower(Math.abs(speed));
                robot.rightDrive2.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftDrive.getCurrentPosition(),
                            robot.rightDrive.getCurrentPosition());
                    telemetry.addData("Back wheels", newLeftTarget2 + "" + newRightTarget2);
                    telemetry.update();
                }

                // Stop all motion
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftDrive2.setPower(0);
                robot.rightDrive2.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        } catch (TargetPositionNotSetException e) {
            telemetry.addData("Mission Failed", "We'll get 'em next time: " + test);
            telemetry.update();
        }
    }
}
