package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Iron Golem Autonomous", group="Mechanum Bot")
public class AutonomousOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private AutonomousMotor frontLeftDrive = null;
    private AutonomousMotor frontRightDrive = null;
    private AutonomousMotor backLeftDrive = null;
    private AutonomousMotor backRightDrive = null;

    private final double widthOfRobot = 0;
    private final double robotTurningCircumference = widthOfRobot * 2 * Math.PI;

    private final AutonomousMotor[] movement = {frontLeftDrive,frontRightDrive,backLeftDrive,backRightDrive};
    private double[] inchCounts = new double[4];


    @Override
    public void runOpMode() throws InterruptedException {
        //INSERT AUTONOMOUS CODE HERE. INITIALIZE AUTONOMOUS MOTORS.
        DcMotor frntLft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frntRght = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor bckLft = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor bckRght = hardwareMap.get(DcMotor.class, "back_right");

        frntLft.setDirection(DcMotor.Direction.REVERSE);
        frntRght.setDirection(DcMotor.Direction.FORWARD);
        bckLft.setDirection(DcMotor.Direction.REVERSE);
        bckRght.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive  = new AutonomousMotor(frntLft, 2240, 4 );
        frontRightDrive = new AutonomousMotor(frntRght, 2240, 4 );
        backLeftDrive  = new AutonomousMotor(bckLft, 2240, 4 );
        backRightDrive = new AutonomousMotor(bckRght, 2240, 4 );

        //EXAMPLE AUTONOMOUS CODE

        //Move Robot forward 48 inches at 80% speed
        inchCountChange(48,48,48,48);
        autonomousCommand(movement,inchCounts,0.8, 3);

        //Move Robot backwards 48 inches at 50% speed
        inchCountChange(-48,-48,-48,-48);
        autonomousCommand(movement,inchCounts,0.5, 3);

        //Turn Robot 90 Degrees at 100% speed
        inchCountTurn(90);
        autonomousCommand(movement,inchCounts,1, 3);

        //Move 18 Inches at -18 Degrees at 70% speed
        inchCountMove(-18,18);
        autonomousCommand(movement,inchCounts,0.7, 3);

    }


    /**
     * autonomousCommand moves all provided motors and servos.
     * @param args a list of all motors to turn
     * @param inches the inches that each motor will move the robot. This array corresponds to args.
     * @param speed a value between 0 and 1 that represents the speed the motors move at to reach their goal
     * @param timeout a minimum time that an autonomousCommand must take up.
     */
    public void autonomousCommand(AutonomousMotor[] args, double[] inches, double speed, double timeout) {

        if (opModeIsActive()) {

            //Activate each Motor
            for (int i = 0; i<args.length; i++) {
                AutonomousMotor currentMotor = args[i];
                int newTarget = currentMotor.motor.getCurrentPosition() + (int) (inches[i] * currentMotor.COUNTS_PER_INCH);
                currentMotor.motor.setTargetPosition(newTarget);
                currentMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentMotor.motor.setPower(speed);
            }

            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < timeout)) {
                //Checks if Any Motors are still Running. If so, it ends the loop early.
                boolean allMotorsFree = true;
                for (AutonomousMotor currentMotor : args) {
                    if (currentMotor.motor.isBusy()) {
                        allMotorsFree = false;
                        break;
                    }
                }
                if (allMotorsFree) {
                    break;
                }

                telemetry.addData("Robot Status", "Motor1 Position %7d", args[1].motor.getCurrentPosition());
                telemetry.update();

            }

            //Stop Each Motor
            for (AutonomousMotor currentMotor : args) {
                currentMotor.motor.setPower(0);
                currentMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

    }

    /**
     * inchCountChange modifies the array "inchCounts"
     */
    public void inchCountChange(double topLeft, double topRight, double bottomLeft, double bottomRight){
        inchCounts[0] = topLeft;
        inchCounts[1] = topRight;
        inchCounts[2] = bottomLeft;
        inchCounts[3] = bottomRight;
    }
    /**
     * inchCountTurn modifies the array "inchCounts" to have values to allow a turn of "degrees" degrees
     */
    public void inchCountTurn(double degrees){
        double turningInches = robotTurningCircumference * (degrees/360);
        turningInches /= 2;
        inchCounts[0] = turningInches;
        inchCounts[1] = -turningInches;
        inchCounts[2] = turningInches;
        inchCounts[3] = -turningInches;
    }
    /**
     * inchCountMove modifies the array "inchCounts" to have values to allow a movement of "degrees" degrees
     */
    public void inchCountMove(double degrees, double inches){
        double robotAngle = Math.toRadians(degrees) + Math.PI / 4;
        inchCounts[0] = inches * Math.cos(robotAngle);
        inchCounts[1] = inches * Math.sin(robotAngle);
        inchCounts[2] = inches * Math.sin(robotAngle);
        inchCounts[3] = inches * Math.cos(robotAngle);
    }
}



