package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


public class Pushbot_2020 {
    public DcMotor driveLF;
    public DcMotor driveRF;
    public DcMotor driveLB;
    public DcMotor driveRB;
    public DcMotor intake;
    //public CRServo testServo;
    //public CRServo spool;
    public DcMotor armMotor;
    private Servo clawServo;
    //public Servo pusherL;
    //public Servo pusherR;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: AndyMark Orbital 20 Motor Encoder from Video
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP AndyMark Orbital 20
    static final double WHEEL_DIAMETER_INCHES = 3.75;     // For figuring circumference
    static final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public Pushbot_2020() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //Define and Initialize servos
        //testServo = hwMap.get(CRServo.class, "testServo");
        //spool = hwMap.get(CRServo.class, "spool");
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        clawServo = hwMap.get(Servo.class, "clawServo");
        //pusherL = hwMap.get(Servo.class, "pusherL");
        //pusherR = hwMap.get(Servo.class, "pusherR");
        // Define and Initialize Motors
        driveLF = hwMap.get(DcMotor.class, "leftFrontDrive");
        driveRF = hwMap.get(DcMotor.class, "rightFrontDrive");
        driveLB = hwMap.get(DcMotor.class, "leftBackDrive");
        driveRB = hwMap.get(DcMotor.class, "rightBackDrive");
        intake = hwMap.get(DcMotor.class, "intake");
        // Set motor direction
        driveLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRF.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        driveLB.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRB.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        intake.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        driveLF.setPower(0);
        driveRF.setPower(0);
        driveRB.setPower(0);
        driveLB.setPower(0);
        intake.setPower(0);
        //set servo to starting position
        //testServo.setPower(0.0);
        //pool.setPower(0.0);
        armMotor.setPower(0.0);
        clawServo.setPosition(0.0);
        //pusherL.setPosition(0.0);
        //pusherR.setPosition(0.0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openClaw(boolean open) {
        if (open) {
            clawServo.setPosition(0);
        } else {
            clawServo.setPosition(1);
        }
    }

    public void armOut(boolean out) {
        double TIMEOUT = 1.0;
        if (out) {
            runtime.reset();
            while (runtime.seconds() < TIMEOUT) {
                armMotor.setPower(-1);
            }
            armMotor.setPower(0);
        } else {
            runtime.reset();
            while (runtime.seconds() < TIMEOUT) {
                armMotor.setPower(1);
            }
            armMotor.setPower(0);
        }
    }

    public void encoderDrive(double speed,
                             double dist, // in inches
                             char dir /*Options: F, B, L, R*/) {
        int TIMEOUT = 10;

        int target;
        // Ensure that the opmode is still active
        try {
            // Determine new target position, and pass to motor controller
            target = (int) (dist * COUNTS_PER_INCH);
            // Decide which direction each motor should go
            if (dir == 'F') {
                driveLF.setTargetPosition(target);
                driveRF.setTargetPosition(target);
                driveLB.setTargetPosition(target);
                driveRB.setTargetPosition(target);
            } else if (dir == 'B') {
                driveLF.setTargetPosition(-target);
                driveRF.setTargetPosition(-target);
                driveLB.setTargetPosition(-target);
                driveRB.setTargetPosition(-target);
            } else if (dir == 'L') {
                driveLF.setTargetPosition(-target);
                driveRF.setTargetPosition(target);
                driveLB.setTargetPosition(target);
                driveRB.setTargetPosition(-target);
            } else if (dir == 'R') {
                driveLF.setTargetPosition(target);
                driveRF.setTargetPosition(-target);
                driveLB.setTargetPosition(-target);
                driveRB.setTargetPosition(target);
            }
            // Turn On RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            driveLF.setPower(Math.abs(speed));
            driveRF.setPower(Math.abs(speed));
            driveLB.setPower(Math.abs(speed));
            driveRB.setPower(Math.abs(speed));
            while (
                    (runtime.seconds() < TIMEOUT) &&
                            (driveLF.isBusy() &&
                                    driveRF.isBusy() &&
                                    driveRB.isBusy() &&
                                    driveLB.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", target, target);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        driveLF.getCurrentPosition(),
                        driveRF.getCurrentPosition());
                telemetry.addData("Back wheels", target + "" + target);
                telemetry.update();
            }

            // Stop all motion
            driveLF.setPower(0);
            driveRF.setPower(0);
            driveLB.setPower(0);
            driveRB.setPower(0);
            // Turn off RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

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
            // Determine new target position, and pass to motor controller
            target = (int) (ang * COUNTS_PER_DEGREE);
            // Decide which direction each motor should go
            if (cWise) {
                driveLF.setTargetPosition(target);
                driveRF.setTargetPosition(-target);
                driveLB.setTargetPosition(target);
                driveRB.setTargetPosition(-target);
            } else {
                driveLF.setTargetPosition(-target);
                driveRF.setTargetPosition(target);
                driveLB.setTargetPosition(-target);
                driveRB.setTargetPosition(target);
            }
            // Turn On RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            driveLF.setPower(Math.abs(speed));
            driveRF.setPower(Math.abs(speed));
            driveLB.setPower(Math.abs(speed));
            driveRB.setPower(Math.abs(speed));
            while ((runtime.seconds() < TIMEOUT) &&
                    (driveLF.isBusy() &&
                            driveRF.isBusy() &&
                            driveRB.isBusy() &&
                            driveLB.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", target, target);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        driveLF.getCurrentPosition(),
                        driveRF.getCurrentPosition());
                telemetry.addData("Back wheels", target + "" + target);
                telemetry.update();
            }

            // Stop all motion
            driveLF.setPower(0);
            driveRF.setPower(0);
            driveLB.setPower(0);
            driveRB.setPower(0);
            // Turn off RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        } catch (TargetPositionNotSetException e) {
            telemetry.addData("Mission Failed", "We'll get 'em next time: ");
            telemetry.update();
        }
    }
}
