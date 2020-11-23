package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Pushbot_2020 {
    public DcMotor driveLF;
    public DcMotor driveRF;
    public DcMotor driveLB;
    public DcMotor driveRB;
    /*public DcMotor vertExt;
    public DcMotor tapeExt;
    public Servo foundHook1;
    public Servo foundHook2;
    public Servo rightClaw;
    public Servo leftClaw;
    public CRServo tuckAwayClaw1;
    public CRServo tuckAwayClaw2;
    public DcMotor stoneClaw;*/
    public CRServo testServo;

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period  = new ElapsedTime();
    /* Constructor */
    public Pushbot_2020(){

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //Define and Initialize servos
        //testServo  = hwMap.get(CRServo.class, "testServo");
        //set servo to starting position
        //testServo.setPower(0);
        // Define and Initialize Motors
        driveLF = hwMap.get(DcMotor.class, "leftFrontDrive");
        driveRF = hwMap.get(DcMotor.class, "rightFrontDrive");
        driveLB = hwMap.get(DcMotor.class, "leftBackDrive");
        driveRB = hwMap.get(DcMotor.class, "rightBackDrive");
        //vertExt = hwMap.get(DcMotor.class, "vertExt");
        //tapeExt = hwMap.get(DcMotor.class, "tapeExt");
        //stoneClaw = hwMap.get(DcMotor.class, "stoneClaw");
        //Set motor direction
        driveLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRF.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        driveLB.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRB.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //vertExt.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //tapeExt.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //stoneClaw.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        driveLF.setPower(0);
        driveRF.setPower(0);
        driveRB.setPower(0);
        driveLB.setPower(0);
        //vertExt.setPower(0);
        //tapeExt.setPower(0);
        //stoneClaw.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //vertExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //stoneClaw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
