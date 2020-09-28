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
    private DcMotor horizontalSlideMotor = null;
    private DcMotor verticalSlideMotor = null;

    private Servo gripper = null;
    private boolean gripperButtonHeld = false;
    private boolean returningSlides = false;



    private final double openGripperPaosition = 0.4; // Percentage of servo turn when open
    private final double closedGripperPosition = 0; // Percentage of servo turn when closed

    private final double turnSensitvity = 0.5; // Multiplier to Turn Speed

    private final double powerMultiplier = 1; // Multiplier to the Movement Speed of Robot
    private final double buildModeSensitivity = 0.25; //Multipler to Movement while holding "B"

    private final double slideSpeed = 0.6; // Multiplier to the Speed of Retracting and Extending the Slides

    private final int maxHorizontalPosition = 0; //The farthest the arm can extend forward.
    private final int maxVerticalPosition = 0; //The furthest the arm can extend upwrd.


    @Override
    public void init() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        horizontalSlideMotor = hardwareMap.get(DcMotor.class, "horiz");
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "vert");
        gripper = hardwareMap.get(Servo.class, "gripper");

        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        horizontalSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        verticalSlideMotor.setDirection(DcMotor.Direction.FORWARD);

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


        // Movement
        //------------
        if(gamepad1.b) //Slow Down Movement Speed if B is held Down
            moveRobot(gamepad1.left_stick_x * buildModeSensitivity, gamepad1.left_stick_y * buildModeSensitivity,gamepad1.right_stick_x * turnSensitvity);
        else
            moveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y,gamepad1.right_stick_x * turnSensitvity);

        // Controlling Linear Slides
        //-------------------------
        if(gamepad1.x && !returningSlides){ //Return All Slides to 0 and Open Gripper if X is pressed
            horizontalSlideMotor.setTargetPosition(0);
            horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            horizontalSlideMotor.setPower(1);

            verticalSlideMotor.setTargetPosition(0);
            verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            verticalSlideMotor.setPower(1);

            gripper.setPosition(openGripperPosition);

            returningSlides = true; //Lock Normal Slide Control Until Slides are finished moving
        }
        else if(returningSlides && !verticalSlideMotor.isBusy() && !horizontalSlideMotor.isBusy()){
            returningSlides = false; // Allow Manual Control of Slides again if both slides are done moving
            horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{ //Normal Control of Slides
            // Vertical Slide
            if (gamepad1.right_trigger > 0.2 && verticalSlideMotor.getCurrentPosition() < maxVerticalPosition)
                verticalSlideMotor.setPower(slideSpeed);
            else if (gamepad1.right_bumper && verticalSlideMotor.getCurrentPosition() > 0)
                verticalSlideMotor.setPower(-slideSpeed);
            else
                verticalSlideMotor.setPower(0.0);

            // Horizontal Slide
            if (gamepad1.left_trigger > 0.2 && horizontalSlideMotor.getCurrentPosition() < maxHorizontalPosition) {
                verticalSlideMotor.setPower(slideSpeed);
            } else if (gamepad1.left_bumper && verticalSlideMotor.getCurrentPosition() > 0)
                verticalSlideMotor.setPower(-slideSpeed);
            else
                verticalSlideMotor.setPower(0.0);


            //Activate Gripper
            if (gamepad1.a && !gripperButtonHeld) { //Only activate the first frame button is held down
                gripperButtonHeld = true;
                if (gripper.getPosition() == closedGripperPosition)
                    gripper.setPosition(openGripperPosition);
                else
                    gripper.setPosition(closedGripperPosition);
            } else if (!gamepad1.a)
                gripperButtonHeld = false; // Allow another gripper activation after button has been released;

        }

        telemetry.addData("vertical pulley positions", "horizontal: %d, vertical: %d", horizontalSlideMotor.getCurrentPosition(), verticalSlideMotor.getCurrentPosition());
    }




    @Override
    public void stop() {
        //horizontalSlideMotor.setTargetPosition(horizontalMin);
        //horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //horizontalSlideMotor.setPower(1);
        //verticalSlideMotor.setTargetPosition(verticalMin);
        //verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //verticalSlideMotor.setPower(1);

        //while(horizontalSlideMotor.isBusy() || verticalSlideMotor.isBusy()){}


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
        double robotAngle = Math.atan2(xComponent,yComponent) + Math.PI / 4;
        frontLeftDrive.setPower((r * Math.cos(robotAngle) + turn) * powerMultiplier);
        frontRightDrive.setPower((r * Math.sin(robotAngle) - turn) * powerMultiplier);
        backLeftDrive.setPower((r * Math.sin(robotAngle) + turn) * powerMultiplier);
        backRightDrive.setPower((r * Math.cos(robotAngle) - turn) * powerMultiplier);
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
        return Math.pow(800.0,input-1);
    }
}