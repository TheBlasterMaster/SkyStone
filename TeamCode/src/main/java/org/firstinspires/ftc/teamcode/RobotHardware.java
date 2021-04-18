package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

public class RobotHardware {
    // ------ HARDWARE
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor intakeMotor = null;
    public DcMotor intakeMotor2 = null;
    public DcMotorEx flyWheel = null;
    public DcMotor wobbleArm = null;

    public Servo trigger = null;
    public Servo gripper = null;
    public Servo funnel = null;
    public Servo funnel_trigger = null;
    public Servo funnel2 = null; 
    
    public ColorSensor top = null;
    public ColorSensor bottom = null;
    // ------



    public void init() {
        //Obtain Hardware Refrences
        //--------------------------
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
        flyWheel = (DcMotorEx)(hardwareMap.get(DcMotor.class, "flywheel"));
        wobbleArm = hardwareMap.get(DcMotor.class, "wobble");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        wobbleArm.setDirection(DcMotor.Direction.FORWARD);


        funnel_trigger = hardwareMap.get(Servo.class, "funnel_trigger");

        trigger = hardwareMap.get(Servo.class, "trigger");
        gripper = hardwareMap.get(Servo.class, "wobble_servo");
        funnel = hardwareMap.get(Servo.class, "funnel");
        funnel2 = hardwareMap.get(Servo.class, "funnel2");
        funnel_trigger = hardwareMap.get(Servo.class, "funnel_trigger");

        gripper.setDirection(Servo.Direction.REVERSE);
        funnel.setDirection(Servo.Direction.FORWARD);
        funnel_trigger.setDirection(Servo.Direction.FORWARD);
        funnel2.setDirection(Servo.Direction.REVERSE);
        
        top = hardwareMap.get(ColorSensor.class, "top");
        bottom = hardwareMap.get(ColorSensor.class, "bottom");
        //=========================
        
        //Set Run Modes
        flyWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set PID Coeffecients of FlyWheel Motor
        PIDCoefficients pidOrig = flyWheel.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients newPID = new PIDCoefficients(30,3,0);
        flyWheel.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, newPID);
        
        telemetry.addData("PID ", "" + pidOrig.p + " " + pidOrig.i + " " + pidOrig.d);
    }
}