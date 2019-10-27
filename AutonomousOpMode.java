package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Iron Golem Autonomous", group="Pushbot")
public class AutonomousOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        //INSERT AUTONOMOUS CODE HERE. INITIALIZE AUTONOMOUS MOTORS.
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

}



