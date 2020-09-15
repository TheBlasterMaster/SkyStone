package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Iron Golem Autonomous", group="Mechanum Bot")
public class AutonomousOpMode extends LinearOpMode {
    //ElaspedTime Object allows us to use Time (Unused in this program)
    private ElapsedTime runtime = new ElapsedTime();

    //Initalizing 4 Drive Train Motors
    DcMotor frntLft = hardwareMap.get(DcMotor.class, "front_left");
    DcMotor frntRght = hardwareMap.get(DcMotor.class, "front_right");
    DcMotor bckLft = hardwareMap.get(DcMotor.class, "back_left");
    DcMotor bckRght = hardwareMap.get(DcMotor.class, "back_right");

    //Set The right motors to be reversed to compensate for their orientation
    frntLft.setDirection(DcMotor.Direction.FORWARD);
    frntRght.setDirection(DcMotor.Direction.REVERSE);
    bckLft.setDirection(DcMotor.Direction.FORWARD);
    bckRght.setDirection(DcMotor.Direction.REVERSE);

    //Creates New Autonomous Wheel Objects (The file to this class is in this same folder).  
    //Does the calculations for how many turns motor needs to do for robot to go X inches forward
    private AutonomousWheelMotor frontLeftDrive = new AutonomousWheelMotor(frntLft, 2240, 2.95276);
    private AutonomousWheelMotor frontRightDrive = new AutonomousWheelMotor(frntRght, 2240, 2.95276);
    private AutonomousWheelMotor backLeftDrive = new AutonomousWheelMotor(bckLft, 2240, 2.95276);
    private AutonomousWheelMotor backRightDrive = new AutonomousWheelMotor(bckRght, 2240, 2.95276);
    
    // Initalizes Servos for Foundation Grippers
    private Servo foundationGripper1 = hardwareMap.get(Servo.class, "gripper1");
    private Servo foundationGripper2 = hardwareMap.get(Servo.class, "gripper2");
    
    //When a robot turns in place, its wheels trace a circle. The Diameter of the Circle is the distance between the wheels
    //The wheels have to move the distance along the circle to turn. The follwoing calculates that.
    private final double widthOfRobot = 16;
    private final double robotTurningCircumference = widthOfRobot * Math.PI;

    //Compensates for the gear ratio of the wheels and for slippage in the wheels
    private final double gearAndFrictionConstant = 8.0/27;

    //The Autonomous Movement Command has been programmed to work with an array of motors so that it can work with
    //any amount of motors. The movement array is an array of all 4 of our drivetrain motors.
    private AutonomousWheelMotor[] movement = {frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};

    //------
    @Override
    public void runOpMode() throws InterruptedException { 
        //EXAMPLE AUTONOMOUS CODE

        //Move Robot forward 8 inches at 50% speed
        manualMove(8,8,8,8,0.5);

        //Turn Robot 90 Degrees at 50% speed
        turn(90,0.5);

        //Move 8 Inches at -18 Degrees at 50% speed
        move(-18,8,0.5);
        
        //NEW (ACTIVATE FOUNDATION GRIPPER)
        foundationGripper1.setPosition(0.2);
        foundationGripper2.setPosition(0.2);
    }


    /**
     * autonomousMovementCommand moves all provided motors and servos while taking into mind gear rations and friction.
     * @param args a list of all motors to turn
     * @param inches the inches that each motor will move the robot. This array corresponds to args.
     * @param speed a value between 0 and 1 that represents the speed the motors move at to reach their goal
     */
    public void autonomousMovementCommand(AutonomousWheelMotor[] args, double[] inches, double speed) {

        //Delted Code: if (opModeIsActive()) {

        //Holds the target encoder value for every motor
        double targets[] = new double[args.length];

        //Sets every motor to run to the correct end position
        for (int i = 0; i<args.length; i++) {
            AutonomousWheelMotor currentMotor = args[i];

            int newTarget = currentMotor.motor.getCurrentPosition() + (int) (inches[i] * gearAndFrictionConstant * currentMotor.COUNTS_PER_INCH);
            targets[i] = (int) (inches[i] * currentMotor.COUNTS_PER_INCH);
            currentMotor.motor.setTargetPosition(newTarget);
            currentMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //Turns every number into a proportional number between 0-1. Used for the speed values, and makes every motor finish at the same time
        targets = divideByLargestInArray(targets,speed);

        //Set The Speeds to each motor
        for (int i = 0; i<args.length; i++) {
            AutonomousWheelMotor currentMotor = args[i];
            currentMotor.motor.setPower(targets[i]);
        }

        //Wait for all motors to finish activity
        //Delted Code: whileOpModeIsActive
        while (args[0].motor.isBusy()) {

        }

        //Stop Each Motor
        for (AutonomousWheelMotor currentMotor : args) {
            currentMotor.motor.setPower(0);
            currentMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //}

    }
    /**
     * autonomousCommand moves all provided motors and servos.
     * @param args a list of all motors to turn
     * @param inches the inches that each motor will move the robot. This array corresponds to args.
     * @param speed a value between 0 and 1 that represents the speed the motors move at to reach their goal
     */
    public void autonomousCommand(AutonomousWheelMotor[] args, double[] inches, double speed) {

        //Deleted: if (opModeIsActive()) {

        //Calculate Speed and Encoder Target for each motor
        double targets[] = new double[args.length];

        //Sets every motor to run to the correct end position
        for (int i = 0; i<args.length; i++) {
            AutonomousWheelMotor currentMotor = args[i];

            int newTarget = currentMotor.motor.getCurrentPosition() + (int) (inches[i] * currentMotor.COUNTS_PER_INCH);
            targets[i] = (int) (inches[i] * currentMotor.COUNTS_PER_INCH);
            currentMotor.motor.setTargetPosition(newTarget);
            currentMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //Turns every number into a proportional number between 0-1. Used for the speed values, and makes every motor finish at the same time
        targets = divideByLargestInArray(targets,speed);

        //Set The Speeds to each motor
        for (int i = 0; i<args.length; i++) {
            AutonomousWheelMotor currentMotor = args[i];
            currentMotor.motor.setPower(targets[i]);
        }

        //Wait for all motors to finish activity
        //Deleted : whileOpModeIsActive
        while (args[0].motor.isBusy()) {

        }

        //Stop Each Motor
        for (AutonomousWheelMotor currentMotor : args) {
            currentMotor.motor.setPower(0);
            currentMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //}

    }

    /**
     * manualMove turns each individual wheel the given amount of inches.
     * @param topLeft the amount of inches the top left wheel covers
     * @param topRight the amount of inches the top left wheel covers
     * @param bottomLeft the amount of inches the top left wheel covers
     * @param bottomRight the amount of inches the top left wheel covers
     * @param speed the speed of all four wheels (0-1)
     */
    private void manualMove(double topLeft, double topRight, double bottomLeft, double bottomRight, double speed){
        double inches[] = {topLeft,topRight,bottomLeft,bottomRight};
        autonomousMovementCommand(movement,inches,speed);
    }

    /**
     * turn rotates the robot around it's center.
     * @param degrees the amount of rotation relavtive the current robots rotation
     * @param speed the speed of turning (0-1)
     */
    private void turn(double degrees,double speed){
        double turningInches = robotTurningCircumference * (degrees/360);
        turningInches /= 2;
        double inches[] = {turningInches,-turningInches,turningInches,-turningInches};
        autonomousCommand(movement,inches,speed);
    }

    /**
     * move strafes the robot in a given amount of degrees
     * @param degrees the direction of movement relative to the robot's current rotation
     * @param inches the distance of movement
     * @param speed the speed of movement (0-1)
     */
    private void move(double degrees, double inches, double speed){
        double robotAngle = Math.toRadians(-degrees) + Math.PI / 4;
        double inchArray[] = {inches * Math.cos(robotAngle),
                              inches * Math.sin(robotAngle),
                              inches * Math.sin(robotAngle),
                              inches * Math.cos(robotAngle)};
        autonomousMovementCommand(movement,inchArray,speed);

    }

    /**
     * Helper method that converts an array into a 0-1 format
     */
    private double[] divideByLargestInArray(double[] array, double multiplier){
        double largest = array[0];
        for(double number: array)
            if(number>largest)
                largest = number;

        for(int i = 0; i<array.length; i++)
            array[i] = (array[i]/largest) * multiplier;

        return array;

    }


}



