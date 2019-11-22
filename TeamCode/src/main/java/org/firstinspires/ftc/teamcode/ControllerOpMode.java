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
    private DcMotor horizontalPulley = null;
    private DcMotor verticalPulley = null;

    private Servo gripper = null;

    private int horizontalMin;
    private int verticalMin;
    private int gripperCoolDown = 0;
    private int verticalCoolDown = 0;
    private int horizontalCoolDown = 0;



    private int verticalTarget = 0;
    private int horizontalTarget = 0;

    //CONTROLLABLE VARIABLES
    private final double powerMultiplier = 1; // Total Speed of Robot
    private final double openGripperPosition = 0.8; // Percentage of servo turn when open
    private final double closedGripperPosition = 0.2; // percentage of servo turn when closed
    private final double turnSensitvity = 0.5; // Turn sensitivity

    private final int horizontalBlockRotations = 250; // amount of horizontal slide movement per button tap
    private final int verticalBlockRotations = 250;// vertical slide movement per button tap

    private final int gripperControlPoint = 120; // amount of time before you can grip again
    private final int horizontalControlPoint = 120; // amount of time before slide can move again
    private final int verticalControlPoint = 120; // amount of time before slide can move again

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        horizontalPulley = hardwareMap.get(DcMotor.class, "horiz");
        verticalPulley = hardwareMap.get(DcMotor.class, "vert");
        gripper = hardwareMap.get(Servo.class, "gripper");


        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        horizontalPulley.setDirection(DcMotor.Direction.REVERSE);
        verticalPulley.setDirection(DcMotor.Direction.FORWARD);



        horizontalMin = horizontalPulley.getCurrentPosition();
        verticalMin = verticalPulley.getCurrentPosition();







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

        double v1;
        double v2;
        double v3;
        double v4;
        /*---------------------------
         *         ______
         *     v1 0|    |0 v2
         *         |    |
         *     v3 0|    |0 v4
         *         ``````
         *Control Scheme:
         *   Left Stick: Movement
         *   Right Stick: Turning
         *   LT and LB: Move Arm Up and Down
         *   A: Grab and Release block
         *   RT and RB: Extend Arm
         ---------------------------*/


        //Movement
        //-------------
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) + Math.PI / 4;
        double rightX = gamepad1.right_stick_x * turnSensitvity;
        v1 = r * Math.cos(robotAngle) + rightX;
        v2 = r * Math.sin(robotAngle) - rightX;
        v3 = r * Math.sin(robotAngle) + rightX;
        v4 = r * Math.cos(robotAngle) - rightX;

        /*
        Scaling Version
        double speed = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        v1 = speed + turn - strafe;
        v2 = speed - turn + strafe;
        v3 = speed + turn + strafe;
        v4 = speed - turn - strafe;

        //Scale variables to value between 0-1
        double[] scalingArray = {v1,v2,v3,v4};
        Arrays.sort(scalingArray);

        v1/= scalingArray[3];
        v2/= scalingArray[3];
        v3/= scalingArray[3];
        v4/= scalingArray[3];
        */

        //Controlling Linear Slides
        //-------------------------
        if (horizontalCoolDown != 0) {
            horizontalCoolDown += 1;
            if (horizontalCoolDown == horizontalControlPoint)
                horizontalCoolDown = 0;
        }

        if (verticalCoolDown != 0) {
            verticalCoolDown += 1;
            if (verticalCoolDown == verticalControlPoint)
                verticalCoolDown = 0;
        }

        // - Horizontal Slide
        if (gamepad1.right_trigger > 0.2 && horizontalTarget!=2 && horizontalCoolDown == 0){
            horizontalTarget+=1;
            horizontalCoolDown+=1;
            horizontalPulley.setTargetPosition(horizontalTarget * horizontalBlockRotations + horizontalMin);
            horizontalPulley.setPower(1);
            horizontalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        if (gamepad1.right_bumper && horizontalTarget !=0 && horizontalCoolDown == 0){
            horizontalTarget-=1;
            horizontalCoolDown+=1;
            horizontalPulley.setTargetPosition(horizontalTarget * horizontalBlockRotations + horizontalMin);
            horizontalPulley.setPower(1);
            horizontalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        // - Verical Slide
        if (gamepad1.left_trigger > 0.2 && verticalTarget !=3 && verticalCoolDown == 0){
            verticalTarget +=1;
            verticalCoolDown+=1;
            verticalPulley.setTargetPosition(verticalTarget * verticalBlockRotations + verticalMin);
            verticalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            verticalPulley.setPower(1);
        }

        if (gamepad1.left_bumper && verticalTarget != 0 && verticalCoolDown == 0)
        {
            verticalTarget -= 1;
            verticalCoolDown+=1;
            verticalPulley.setTargetPosition(verticalTarget * verticalBlockRotations + verticalMin);
            verticalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            verticalPulley.setPower(1);
        }

        if (gripperCoolDown != 0) {
            gripperCoolDown += 1;
            if (gripperCoolDown == gripperControlPoint)
                gripperCoolDown = 0;
        }



        //Activate Gripper
        if (gamepad1.a && gripperCoolDown == 0) {
            gripperCoolDown+=1;
            if (gripper.getPosition() == closedGripperPosition)
                gripper.setPosition(openGripperPosition);
            else
                gripper.setPosition(closedGripperPosition);

        }





        frontLeftDrive.setPower(v1 * powerMultiplier);
        frontRightDrive.setPower(v2 * powerMultiplier);
        backLeftDrive.setPower(v3 * powerMultiplier);
        backRightDrive.setPower(v4 * powerMultiplier);

        telemetry.addData("Status", "Run Time: " + horizontalTarget);
        telemetry.addData("Miles is sla e", "Run Time: " + gripperCoolDown);


    }


    @Override
    public void stop() {
        horizontalPulley.setTargetPosition(horizontalMin);
        horizontalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalPulley.setPower(1);
        verticalPulley.setTargetPosition(verticalMin);
        verticalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalPulley.setPower(1);

        while(horizontalPulley.isBusy() || verticalPulley.isBusy()){}


    }

}
