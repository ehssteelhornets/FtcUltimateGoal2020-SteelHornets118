package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


public class Pushbot_2020 {
    public DcMotor driveLF;
    public DcMotor driveRF;
    public DcMotor driveLB;
    public DcMotor driveRB;
    public DcMotor intake;
    public DcMotor launchL;
    public DcMotor launchR;
    public CRServo lift;
    public DcMotor armMotor;
    private Servo clawServo;
    public Servo pusherL;
    public Servo pusherR;
    public Servo barrelServoL;
    public Servo barrelServoR;
    public ModernRoboticsI2cRangeSensor liftSensor;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: AndyMark Orbital 20 Motor Encoder from Video
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP AndyMark Orbital 20
    static final double WHEEL_DIAMETER_INCHES = 3.75;     // For figuring circumference
    static final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Set lift variables
    final int up = -1;
    // Positions are the number of inches between the range sensor and the bottom of the lift
    final double[] liftPos = new double[]{10, 7, 4.88, 2.6}; // Intake position, Launch position for 3 rings, Launch position for 2 rings, Launch position for 1 ring
    public int currLiftPos = 1;

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
        lift = hwMap.get(CRServo.class, "liftServo");
        pusherL = hwMap.get(Servo.class, "leftPushServo");
        pusherR = hwMap.get(Servo.class, "rightPushServo");
        clawServo = hwMap.get(Servo.class, "clawServo");
        barrelServoL = hwMap.get(Servo.class, "barrelL");
        barrelServoR = hwMap.get(Servo.class, "barrelR");
        // Define and Initialize Motors
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        launchL = hwMap.get(DcMotor.class, "leftLaunch");
        launchR = hwMap.get(DcMotor.class, "rightLaunch");
        driveLF = hwMap.get(DcMotor.class, "leftFrontDrive");
        driveRF = hwMap.get(DcMotor.class, "rightFrontDrive");
        driveLB = hwMap.get(DcMotor.class, "leftBackDrive");
        driveRB = hwMap.get(DcMotor.class, "rightBackDrive");
        intake = hwMap.get(DcMotor.class, "intake");
        // Define and Initialize sensors
        liftSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "liftSensor");
        // Set motor direction
        driveLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRF.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        driveLB.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRB.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        intake.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        armMotor.setPower(0);
        rev(false);
        driveLF.setPower(0);
        driveRF.setPower(0);
        driveRB.setPower(0);
        driveLB.setPower(0);
        intake.setPower(0);
        //set servo to starting position
        lift.setPower(0);
        clawServo.setPosition(0.4);
        launch(false);
        barrelServoL.setPosition(.6);
        barrelServoR.setPosition(.4);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openClaw(boolean open) {
        if (open) {
            clawServo.setPosition(.4);
        } else {
            clawServo.setPosition(1);
        }
    }

    public void armOut(boolean out) {
        double TIMEOUT = 1.0;
        if (out) {
            runtime.reset();
            while (runtime.seconds() < TIMEOUT) {
                armMotor.setPower(.5);
            }
            armMotor.setPower(0);
        } else {
            runtime.reset();
            while (runtime.seconds() < TIMEOUT) {
                armMotor.setPower(-.5);
            }
            armMotor.setPower(0);
        }
    }

    public void rev(boolean on) {
        if (on) {
            launchL.setPower(.75);
            launchR.setPower(-.75);
        } else {
            launchR.setPower(0.0);
            launchL.setPower(0.0);
        }
    }

    public void launch(boolean go) {
        if (go) {
            pusherL.setPosition(0.25);
            pusherR.setPosition(0.25);
        } else {
            pusherL.setPosition(1.0);
            pusherR.setPosition(0.025);
        }
    }

    public void barrel(int pos) {
        if (pos == 0) {
            // Ring Pickup Position
            barrelServoR.setPosition(1);
            barrelServoL.setPosition(0);
        } else if (pos == 1) {
            // Power Shot Position
            barrelServoR.setPosition(.8);
            barrelServoL.setPosition(.2);
        } else if (pos == 2) {
            // High Goal Position
            barrelServoR.setPosition(.7);
            barrelServoL.setPosition(.3);
        }
    }

    public void liftDown(double speed) {
        currLiftPos = 0;
        while (getBetterDistance() < liftPos[0]) {
            lift.setPower(-1 * speed * up);
        }
        lift.setPower(0);
    }

    public void liftUp(double speed) {
        currLiftPos++;
        if(currLiftPos > 3) {
            currLiftPos = 3;
        }
        while (getBetterDistance() > liftPos[currLiftPos]) {
            lift.setPower(speed * up);
        }
        lift.setPower(0);
    }

    public double getBetterDistance() {
        double dist = liftSensor.getDistance(DistanceUnit.CM);
        if (!Double.isNaN(dist)) {
            return dist;
        } else {
            return liftSensor.cmUltrasonic();
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
