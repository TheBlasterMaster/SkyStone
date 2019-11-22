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
    //Runtime Object contains timer of how long OpMode has been Running
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor horizontalPulley = null;
    private DcMotor verticalPulley = null;

    //Servos
    private Servo gripper = null;

    //Variables used in the OpMode
    private int horizontalMin; //Store original Motor Position
    private int verticalMin; //Store original Motor Position

    private int gripperCoolDown = 0; //Store "time" since last button press and if ready for another press
    private int verticalCoolDown = 0; //Store "time" since last button press and if ready for another press
    private int horizontalCoolDown = 0; //Store "time" since last button press and if ready for another press

    private int verticalTarget = 0; //Stores current "stage" arm is at.
    private int horizontalTarget = 0;//Stores current "stage" arm is at.

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

        //Instantiate all Motors and Servos
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        horizontalPulley = hardwareMap.get(DcMotor.class, "horiz");
        verticalPulley = hardwareMap.get(DcMotor.class, "vert");
        gripper = hardwareMap.get(Servo.class, "gripper");

        //Set Direction of Every Motor
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        horizontalPulley.setDirection(DcMotor.Direction.REVERSE);
        verticalPulley.setDirection(DcMotor.Direction.FORWARD);

        //Instantiate Mins
        horizontalMin = horizontalPulley.getCurrentPosition();
        verticalMin = verticalPulley.getCurrentPosition();

        //Print Initialization Confirmation
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

        //Controlling Linear Slides
        //-------------------------

        // - Debouncing, prevents multiple inputs with one short button press
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

        // Out
        if (gamepad1.right_trigger > 0.2 && horizontalTarget!=2 && horizontalCoolDown == 0){
            horizontalTarget+=1;
            horizontalCoolDown+=1;
            horizontalPulley.setTargetPosition(horizontalTarget * horizontalBlockRotations + horizontalMin);
            horizontalPulley.setPower(1);
            horizontalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // In
        if (gamepad1.right_bumper && horizontalTarget !=0 && horizontalCoolDown == 0){
            horizontalTarget-=1;
            horizontalCoolDown+=1;
            horizontalPulley.setTargetPosition(horizontalTarget * horizontalBlockRotations + horizontalMin);
            horizontalPulley.setPower(1);
            horizontalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // - Vertical Slide

        //Out
        if (gamepad1.left_trigger > 0.2 && verticalTarget !=3 && verticalCoolDown == 0){
            verticalTarget +=1;
            verticalCoolDown+=1;
            verticalPulley.setTargetPosition(verticalTarget * verticalBlockRotations + verticalMin);
            verticalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            verticalPulley.setPower(1);
        }

        // In
        if (gamepad1.left_bumper && verticalTarget != 0 && verticalCoolDown == 0)
        {
            verticalTarget -= 1;
            verticalCoolDown+=1;
            verticalPulley.setTargetPosition(verticalTarget * verticalBlockRotations + verticalMin);
            verticalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            verticalPulley.setPower(1);
        }

        //Controlling the Gripper
        //-----------------------

        //Debouncing, prevents multiple inputs with one short input
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

        //Activate Motors
        frontLeftDrive.setPower(v1 * powerMultiplier);
        frontRightDrive.setPower(v2 * powerMultiplier);
        backLeftDrive.setPower(v3 * powerMultiplier);
        backRightDrive.setPower(v4 * powerMultiplier);

        //Telemetry
        telemetry.addData("Status", "Run Time: " + horizontalTarget);
        telemetry.addData("Miles is sla e", "Run Time: " + gripperCoolDown);
    }

    @Override
    public void stop() {
        //Return Slides to Normal Position
        horizontalPulley.setTargetPosition(horizontalMin);
        horizontalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalPulley.setPower(1);

        verticalPulley.setTargetPosition(verticalMin);
        verticalPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalPulley.setPower(1);

        //Hold out method until the pulleys finish returning
        while(horizontalPulley.isBusy() || verticalPulley.isBusy()){}

    }

}
