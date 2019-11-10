package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Iron Golem Autonomous", group="Mechanum Bot")
public class AutonomousOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private AutonomousMotor frontLeftDrive = null;
    private AutonomousMotor frontRightDrive = null;
    private AutonomousMotor backLeftDrive = null;
    private AutonomousMotor backRightDrive = null;

    private final double widthOfRobot = 16;
    private final double robotTurningCircumference = widthOfRobot * 2 * Math.PI;

    private AutonomousMotor[] movement = new AutonomousMotor[4];


    @Override
    public void runOpMode() throws InterruptedException {

        //Initializing Motors
        DcMotor frntLft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frntRght = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor bckLft = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor bckRght = hardwareMap.get(DcMotor.class, "back_right");

        frntLft.setDirection(DcMotor.Direction.FORWARD);
        frntRght.setDirection(DcMotor.Direction.REVERSE);
        bckLft.setDirection(DcMotor.Direction.FORWARD);
        bckRght.setDirection(DcMotor.Direction.REVERSE);

        frntLft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frntRght.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bckLft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bckRght.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive  = new AutonomousMotor(frntLft, 2240, 2.95276 );
        frontRightDrive = new AutonomousMotor(frntRght, 2240, 2.95276 );
        backLeftDrive  = new AutonomousMotor(bckLft, 2240, 2.95276 );
        backRightDrive = new AutonomousMotor(bckRght, 2240, 2.95276 );

        movement[0] = frontLeftDrive;
        movement[1] = frontRightDrive;
        movement[2] = backLeftDrive;
        movement[3] = backRightDrive;



        //EXAMPLE AUTONOMOUS CODE

        //Move Robot forward 8 inches at 50% speed
        manualMove(8,8,8,8,0.5);

        //Turn Robot 90 Degrees at 50% speed
        turn(90,0.5);

        //Move 8 Inches at -18 Degrees at 50% speed
        move(-18,8,0.5);

    }


    /**
     * autonomousCommand moves all provided motors and servos.
     * @param args a list of all motors to turn
     * @param inches the inches that each motor will move the robot. This array corresponds to args.
     * @param speed a value between 0 and 1 that represents the speed the motors move at to reach their goal
     */
    public void autonomousCommand(AutonomousMotor[] args, double[] inches, double speed) {

        //if (opModeIsActive()) {

        //Calculate Speed and Encoder Target for each motor
        double targets[] = new double[args.length];

        for (int i = 0; i<args.length; i++) {
            AutonomousMotor currentMotor = args[i];

            int newTarget = currentMotor.motor.getCurrentPosition() + (int) (inches[i] * currentMotor.COUNTS_PER_INCH);
            targets[i] = (int) (inches[i] * currentMotor.COUNTS_PER_INCH);
            currentMotor.motor.setTargetPosition(newTarget);
            currentMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        targets = divideByLargestInArray(targets,speed);
        //Set The Speeds to each motor
        for (int i = 0; i<args.length; i++) {
            AutonomousMotor currentMotor = args[i];
            currentMotor.motor.setPower(targets[i]);
        }

        //Run Each Motor to the Target
        //whileOpModeIsActive
        while (args[0].motor.isBusy()) {

        }

        //Stop Each Motor
        for (AutonomousMotor currentMotor : args) {
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
        autonomousCommand(movement,inches,speed);
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
        autonomousCommand(movement,inchArray,speed);

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
            array[i]= (array[i]/largest) * multiplier;

        return array;

    }


}



