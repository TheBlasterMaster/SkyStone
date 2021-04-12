/* Copyright (c) 2017 FIRST. All rights reserved.
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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


@TeleOp(name="Iron Golem Controller", group="Mechanum Bot")
public class ControllerOpMode extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor intakeMotor2 = null;
    private DcMotorEx flyWheel = null;
    private DcMotor wobbleArm = null;

    private Servo trigger = null;
    private Servo gripper = null;
    private Servo funnel = null;
    private Servo funnel_trigger = null;
    private Servo funnel2 = null; 
    
    private ColorSensor top = null;
    private ColorSensor bottom = null;
    //-------SETTINGS
    private boolean flutterTrigger = false; //One Trigger Press Shoots 3 Rings
    //-------
    
    private boolean flutterTriggerActive = false;
    private final int FLUTTER_SERVOINTIME = 80;
    private final int FLUTTER_SERVOOUTTIME = 60;
    private int flutterStage = 0;
    private final int[] FLUTTER_TIMES = {FLUTTER_SERVOOUTTIME, 
                                        FLUTTER_SERVOOUTTIME + FLUTTER_SERVOINTIME, 
                                        2*FLUTTER_SERVOOUTTIME + FLUTTER_SERVOINTIME,  
                                        2*FLUTTER_SERVOOUTTIME + 2*FLUTTER_SERVOINTIME, 
                                        3*FLUTTER_SERVOOUTTIME + 2*FLUTTER_SERVOINTIME,
                                        3*FLUTTER_SERVOOUTTIME + 3*FLUTTER_SERVOINTIME};
    private int flutterTriggerTimer =0;
    
    private int armMin; //Variable that stores resting position of Arm
    private int gripperState = 2; // 0 = grab; 1= lift; 2 = completelyUp;
    
    private boolean lastUpDpad =false;   //Variables that store state of dpad last frame
    private boolean lastDownDpad =false; //This allows us to determine first frame of input
    
    private double turnSensitvity = 1; // Multiplier to Turn Speed

    private final double powerMultiplier = 1; // Multiplier to the Movement Speed of Robot
    private final double buildModeSensitivity = 0.8; //Multipler to Movement while holding "B"

    private final double intakePower = 1; // Controls speed of the intake rollers

    private final double triggerPos = 0.3; // Percentage of Full Turn Servo Makes to push ring into flywheel
    private final double triggerRest = 0.1; // Percentage of Full Turn Servo Rests at 
    
    private final double gripperPos = 0.3; // Percentage of Full Turn Gripper Makes to grab Wobble Goal
    
    private final int armGrab = 70;
    private final int armLift = 50;
    private final int[] armPositionLookup = {armMin+armGrab,armMin+armLift,armMin};

    private int[] previousFlyWheelPositions = new int[20];
    
    private double previousXStick = 0;
    private double previousYStick = 0;
    private double maxChange = 0.4;
    @Override
    public void init() {
        
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
        flyWheel = (DcMotorEx)(hardwareMap.get(DcMotor.class, "flywheel"));
        wobbleArm = hardwareMap.get(DcMotor.class, "wobble");
        funnel_trigger = hardwareMap.get(Servo.class, "funnel_trigger");
        funnel_trigger.setDirection(Servo.Direction.REVERSE);
        
        flyWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
       flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
       frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        for(int i = 0; i<previousFlyWheelPositions.length;i++)
            previousFlyWheelPositions[i] = 0;
        
        trigger = hardwareMap.get(Servo.class, "trigger");
        gripper = hardwareMap.get(Servo.class, "wobble_servo");
        funnel = hardwareMap.get(Servo.class, "funnel");
        funnel2 = hardwareMap.get(Servo.class, "funnel2");
        funnel_trigger = hardwareMap.get(Servo.class, "funnel_trigger");
        
        top = hardwareMap.get(ColorSensor.class, "top");
        bottom = hardwareMap.get(ColorSensor.class, "bottom");

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        wobbleArm.setDirection(DcMotor.Direction.FORWARD);
        gripper.setDirection(Servo.Direction.REVERSE);
        funnel.setDirection(Servo.Direction.FORWARD);
        funnel_trigger.setDirection(Servo.Direction.FORWARD);
        funnel2.setDirection(Servo.Direction.REVERSE);
        
        
        armMin = wobbleArm.getCurrentPosition();
    
        
        telemetry.addData("Status", "Initialized");
        
        PIDCoefficients pidOrig = flyWheel.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients newPID = new PIDCoefficients(30,3,0);
        flyWheel.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, newPID);
        
        telemetry.addData("PID ", "" + pidOrig.p + " " + pidOrig.i + " " + pidOrig.d);
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        /***---------------------------
         *         ______
         *     v1 0|    |0 v2
         *         |    |
         *     v3 0|    |0 v4
         *         ``````
         *Control Scheme:
         *   Left Stick: Movement
         *   Right Stick: Turning
         *   RT and RB: Move Arm Up and Down
         *   A: Grab and Release block
         *   LT and LB: Extend Arm
         *   B: Build Mode, Slows Down Movement
         *   X: Return Slides to 0 and Open Gripper
         ---------------------------**/
        //int dPadInput = ((gamepad1.dpad_up && !lastUpDpad)? 1:0) - 
                        //((gamepad1.dpad_down && !lastDownDpad)? 1:0);
        int dPadInput = ((gamepad1.dpad_up)? 1:0) - 
                        ((gamepad1.dpad_down)? 1:0);
        
        /**
        if(dPadInput != 0){
            gripperState+=dPadInput;
            gripperState = Math.max(0,Math.min(gripperState,2));
            wobbleArm.setTargetPosition(armPositionLookup[gripperState]);
            wobbleArm.setPower(1);
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        **/
        
        
        wobbleArm.setPower(dPadInput*0.5);
       
        for(int i = 0; i<previousFlyWheelPositions.length-1; i++) // Slides down old values to insert new value
            previousFlyWheelPositions[i] = previousFlyWheelPositions[i+1];
        
        previousFlyWheelPositions[previousFlyWheelPositions.length-1] = flyWheel.getCurrentPosition();
        
        double sum = 0;
        double averageFlyWheelSpeed;
        
        for(int i=0; i<previousFlyWheelPositions.length-1; i++){
            sum += (1.0*(previousFlyWheelPositions[i+1]-previousFlyWheelPositions[i])*60);
        }
        
        averageFlyWheelSpeed = sum/(previousFlyWheelPositions.length-1);
    
        if(gamepad1.a){
            intakeMotor.setPower(intakePower);
            intakeMotor2.setPower(1);
            funnel.setPosition(1);
            funnel2.setPosition(1);
        }
        else if(gamepad1.y){
            intakeMotor.setPower(-intakePower);
            intakeMotor2.setPower(-1);
        }
        else{
            intakeMotor.setPower(0);
            intakeMotor2.setPower(0);
            funnel.setPosition(0.5);
            funnel2.setPosition(0.5);
        }
        
        if(gamepad1.x)
            gripper.setPosition(gripperPos);
        else
            gripper.setPosition(0.02);
        
        if(gamepad1.left_trigger > 0.1){
            if(gamepad1.left_trigger > 0.8)
                flyWheel.setPower(0.56);  
            else
                flyWheel.setPower(0.56);
                
            //flyWheel.setPower(Math.min(gamepad1.left_trigger,0.8));
            //flyWheel.setSpeed(3); //apparently doesn't exist
            //0.75 for second highest
        }
        else
            flyWheel.setPower(0);
        
        if(gamepad1.right_bumper)
            funnel_trigger.setPosition(0.3);
        else    
            funnel_trigger.setPosition(0.04);
            
        if(!flutterTrigger){
            if(gamepad1.right_trigger > 0.1)
                trigger.setPosition(triggerPos);
            else
                trigger.setPosition(triggerRest);
        }
        else{
            if(gamepad1.right_trigger > 0.1 && !flutterTriggerActive){
                flutterTriggerActive = true;
                trigger.setPosition(triggerPos);
                flutterStage = 0;
                flutterTriggerTimer = 0;
            }
            else if(flutterTriggerActive){
                flutterTriggerTimer++;
                if(flutterTriggerTimer >= FLUTTER_TIMES[flutterStage]){
                    if(flutterStage == 5)
                        flutterTriggerActive = false;
                    else if(flutterStage%2 == 0)
                        trigger.setPosition(triggerRest);
                    else
                        trigger.setPosition(triggerPos);
                    flutterStage++;
                }
            }
        }
        
        lastDownDpad = gamepad1.dpad_down;
        lastUpDpad = gamepad1.dpad_up;
        
        bottom.enableLed(false);
        top.enableLed(false);
        
        // Movement
        //------------
        int dPadInputStrafe = ((gamepad1.dpad_right)? 1:0) - 
                        ((gamepad1.dpad_left)? 1:0);
        double leftStickx = Math.min(gamepad1.left_stick_x + dPadInputStrafe,1);
        double leftSticky = gamepad1.left_stick_y;
        
        double xChange = leftStickx - previousXStick;
        double yChange = leftSticky - previousYStick;
        double magnitudeOfChange = Math.sqrt(yChange*yChange + xChange*xChange);
        
        if (magnitudeOfChange > maxChange){
            leftStickx = previousXStick + maxChange*xChange;
            leftSticky = previousYStick + maxChange*yChange;
        }
        
        previousXStick = leftStickx;
        previousYStick = leftSticky;
        
        if(gamepad1.b || gamepad1.x || gamepad1.left_trigger > 0.1){ //Slow Down Movement Speed if B is held Down
            turnSensitvity = 0.5;
            moveRobot(leftStickx * buildModeSensitivity, leftSticky * buildModeSensitivity,gamepad1.right_stick_x * turnSensitvity);
        }
        else{
            turnSensitvity = 1;
            moveRobot(leftStickx, leftSticky ,gamepad1.right_stick_x * turnSensitvity);
        }


        double[] bottomRGB = {bottom.red(), bottom.green(), bottom.blue()};
        double[] topRGB = {top.red(), top.green(), top.blue()};
        
        bottomRGB = normalizeList(bottomRGB,255);
        topRGB = normalizeList(topRGB,255);
        
        telemetry.addData("Stick Data", "X: %d, Y: %d", (int)(gamepad1.left_stick_x*100) , (int)(gamepad1.left_stick_y*100));
        telemetry.addData("Turn", "Right Stick: %d", (int)(gamepad1.right_stick_x*100));
        telemetry.addData("Gripper State", "State: %d", gripperState);
        telemetry.addData("FlyWheel Speed", "Speed: %d", (int)(100*averageFlyWheelSpeed));
        int rings = (bottom.red()>200)? ((top.red()>200)? 4:1) : 0;
        telemetry.addData("Rings Detected", "Rings: %d ", rings);
        //m
    }




    @Override
    public void stop() {
        //horizontalSlideMotor.setTargetPosition(horizontalMin);
        //horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //horizontalSlideMotor.setPower(1);
        //verticalSlideMotor.setTargetPosition(verticalMin);
        //verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //verticalSlideMotor.setPower(1);

        //while(horizontalSlideMotor.isBusy() || verticalSlideMotor.isBusy())

    }

    // PRIVATE METHODS

    /**
     * Sets Appropriate Power Amounts to Wheels
     * @param xComponent X Part of Movement Vector
     * @param yComponent Y Part of Movement Vector
     * @param turn How Much Robot Should Turn
     */
     private void moveRobot(double xComponent,double yComponent, double turn){
        double r = controllerInputAdjustment( Math.hypot(xComponent, yComponent) );
        telemetry.addData("Radius", "R: %d", (int)(r*100));
        double robotAngle = -Math.atan2(xComponent,yComponent) + Math.PI/4 + Math.PI;
        
        double turnAmount = turn*turnSensitvity;
        double[] input = {r*Math.cos(robotAngle),
                          r*Math.sin(robotAngle),
                          r*Math.sin(robotAngle),
                          r*Math.cos(robotAngle)};
                          
        double magnitudeOfMovement = r * powerMultiplier;
        
        double[] proportionList = normalizeList(input, 1 - Math.abs(turnAmount));
        frontLeftDrive.setPower((proportionList[0] * magnitudeOfMovement) + turnAmount);
        frontRightDrive.setPower((proportionList[1] * magnitudeOfMovement) - turnAmount);
        backLeftDrive.setPower((proportionList[2] * magnitudeOfMovement) + turnAmount);
        backRightDrive.setPower((proportionList[3] * magnitudeOfMovement) - turnAmount);
        
    }
    
    
    /**
     * Takes in list of doubles, returns list with biggest original value set to max parameter
     * @param input double array of numbers to be normalized
     * @param max double that dictaces what the biggest number in the list is set to
     */
     private double[] normalizeList (double[] input, double max){
         double biggestdouble = -2;
         for(double e: input)
            if (Math.abs(e)> biggestdouble)
                biggestdouble = Math.abs(e);
        if (biggestdouble != 0){
            biggestdouble/= max; // Multipliying in the max
            for(int i= 0; i<input.length; i++){
                input[i]/= biggestdouble;
            }
        }
        return input;
     }
    /**
     * 
     * @param input Strength of the controller input (hypotenuse of X and Y input)
     * @return input adjusted by a function
     */
    private double controllerInputAdjustment(double input){
        //An exponential function makes controlling much more precise, since most inputs will barely change the speed,
        //and only inputs very faraway from the origin will result in higher speeds.
        
        //800^(x-1)
        double val = Math.pow(30.0,(1.00964*input)-1) - (1/30);
        return (val>0.05)? val : 0;
    }
}