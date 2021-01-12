package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Pushbot_2020 {
    public DcMotor driveLF;
    public DcMotor driveRF;
    public DcMotor driveLB;
    public DcMotor driveRB;
    public DcMotor intake;
    public CRServo testServo;
    public CRServo spool;
    public Servo armServo;
    public Servo clawServo;
    public Servo pusherL;
    public Servo pusherR;

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Pushbot_2020() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //Define and Initialize servos
        testServo = hwMap.get(CRServo.class, "testServo");
        spool = hwMap.get(CRServo.class, "spool");
        armServo = hwMap.get(Servo.class, "armServo");
        clawServo = hwMap.get(Servo.class, "clawServo");
        pusherL = hwMap.get(Servo.class, "pusherL");
        pusherR = hwMap.get(Servo.class, "pusherR");
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
        testServo.setPower(0.0);
        spool.setPower(0.0);
        armServo.setPosition(0.0);
        clawServo.setPosition(0.0);
        pusherL.setPosition(0.0);
        pusherR.setPosition(0.0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
