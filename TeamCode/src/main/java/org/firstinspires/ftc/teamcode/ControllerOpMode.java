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

    //CONTROLLABLE VARIABLES
    private final double powerMultiplier = 0.8;
    private final double openGripperPosition = 0.8;
    private final double closedGripperPosition = 0.5;


    private final int horizontalBlockRotations = 2204;
    private final int verticalBlockRotations = 2204;

    private final double linearSlideControlPoint = 0.2;
    private final int gripperControlPoint = 60;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        horizontalPulley = hardwareMap.get(DcMotor.class, "horiz");
        verticalPulley = hardwareMap.get(DcMotor.class, "vert");


        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        horizontalPulley.setDirection(DcMotor.Direction.REVERSE);
        verticalPulley.setDirection(DcMotor.Direction.REVERSE);

        horizontalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        horizontalMin = horizontalPulley.getCurrentPosition();
        horizontalPulley.setTargetPosition(horizontalPulley.getCurrentPosition());
        verticalMin = verticalPulley.getCurrentPosition();
        verticalPulley.setTargetPosition(horizontalPulley.getCurrentPosition());



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
        /***---------------------------
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
         ---------------------------**/


        //Movement
        //-------------
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y) + Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
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
        // - Horizontal Slide
        if (gamepad1.right_trigger > 0.2 &&
                (horizontalPulley.getCurrentPosition() - horizontalMin) % horizontalBlockRotations > horizontalBlockRotations * linearSlideControlPoint) {
            int target = (horizontalPulley.getTargetPosition() - horizontalMin) % horizontalBlockRotations + 1;
            if (target != 4) {
                horizontalPulley.setTargetPosition(horizontalMin + (target) * horizontalBlockRotations);
            }
        }

        if (gamepad1.right_bumper &&
                (horizontalPulley.getCurrentPosition() - horizontalMin) % horizontalBlockRotations < horizontalBlockRotations * (1 - linearSlideControlPoint)) {
            int target = (horizontalPulley.getTargetPosition() - horizontalMin) % horizontalBlockRotations - 1;
            if (target != -1) {
                horizontalPulley.setTargetPosition(horizontalMin + (target) * horizontalBlockRotations);
            }
        }
        // - Verical Slide
        if (gamepad1.left_trigger > 0.2 &&
                (verticalPulley.getCurrentPosition() - verticalMin) % verticalBlockRotations > verticalBlockRotations * linearSlideControlPoint) {
            int target = (verticalPulley.getTargetPosition() - verticalMin) % verticalBlockRotations + 1;
            if (target != 4) {
                verticalPulley.setTargetPosition(verticalMin + (target) * verticalBlockRotations);
            }
        }

        if (gamepad1.left_bumper &&
                (verticalPulley.getCurrentPosition() - verticalMin) % verticalBlockRotations < verticalBlockRotations * (1 - linearSlideControlPoint)) {
            int target = (verticalPulley.getTargetPosition() - verticalMin) % verticalBlockRotations - 1;
            if (target != -1) {
                verticalPulley.setTargetPosition(verticalMin + (target) * verticalBlockRotations);
            }
        }

        //Activate Gripper
        if (gamepad1.a && gripperCoolDown == 0) {
            if (gripper.getPosition() == closedGripperPosition)
                gripper.setPosition(openGripperPosition);
            else
                gripper.setPosition(closedGripperPosition);

        }


        if (gripperCoolDown != 0) {
            gripperCoolDown += 1;
            if (gripperCoolDown == gripperControlPoint) {
                gripperCoolDown = 0;
            }


            frontLeftDrive.setPower(v1 * powerMultiplier);
            frontRightDrive.setPower(v2 * powerMultiplier);
            backLeftDrive.setPower(v3 * powerMultiplier);
            backRightDrive.setPower(v4 * powerMultiplier);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
    }

    @Override
    public void stop() {
    }

}
