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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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

    private final double turnSensitvity = 0.7; // Multiplier to Turn Speed

    private final double powerMultiplier = 1; // Multiplier to the Movement Speed of Robot
    private final double buildModeSensitivity = 0.8; //Multipler to Movement while holding "B"

    private final double intakePower = 0.7; // Controls speed of the intake rollers


    @Override
    public void init() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");


        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
         
        telemetry.addData("Status", "Initialized");
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

        intakeMotor.setPower(intakePower);
        // Movement
        //------------
        if(gamepad1.b) //Slow Down Movement Speed if B is held Down
            moveRobot(gamepad1.left_stick_x * buildModeSensitivity, gamepad1.left_stick_y * buildModeSensitivity,gamepad1.right_stick_x * turnSensitvity);
        else
            moveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y,gamepad1.right_stick_x * turnSensitvity);


        telemetry.addData("Stick Data", "X: %d, Y: %d", (int)(gamepad1.left_stick_x*100) , (int)(gamepad1.left_stick_y*100));
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
        double robotAngle = Math.PI - Math.atan2(xComponent,yComponent) + Math.PI/4 + Math.PI ;
        
        double turnAmount = turn*turnSensitvity;
        double[] input = {r*Math.cos(robotAngle),
                          r* Math.sin(robotAngle),
                          r*Math.sin(robotAngle),
                          r*Math.cos(robotAngle)};
                          
        double magnitudeOfMovement = r * powerMultiplier;
        double[] proportionList = normalizeList(input, 1 - Math.abs(turnAmount));
        frontLeftDrive.setPower(proportionList[0] * magnitudeOfMovement + turnAmount);
        frontRightDrive.setPower(proportionList[1] * magnitudeOfMovement - turnAmount);
        backLeftDrive.setPower(proportionList[2] * magnitudeOfMovement + turnAmount);
        backRightDrive.setPower(proportionList[3] * magnitudeOfMovement - turnAmount);
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
        biggestdouble/= max; // Multipliying in the max
        for(int i= 0; i<input.length; i++){
            input[i]/= biggestdouble;
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
        return Math.pow(30.0,(1.00964*input)-1) - (1/30);
    }
}