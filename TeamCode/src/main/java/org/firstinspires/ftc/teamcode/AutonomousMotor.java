package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A class that holds values for the encoders of each motor and the wheel attached to it.
 */
public class AutonomousMotor{
    DcMotor motor = null;

    double COUNTS_PER_MOTOR_REV;
    double WHEEL_DIAMETER_INCHES;
    double COUNTS_PER_INCH;

    /**
     * Initializes the autonomousMotor
     * @param incomingMotor a DcMotor that the Autonomous Motor describes.
     * @param encoderCount the amount of counts that the encoder performs per motor rotation (Google this number).
     * @param diameter the diameter of the wheel attached to the motor.
     */
    public AutonomousMotor(DcMotor incomingMotor, double encoderCount, double diameter){
        motor  = incomingMotor;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        COUNTS_PER_MOTOR_REV = encoderCount;
        WHEEL_DIAMETER_INCHES = diameter;

        COUNTS_PER_INCH=(COUNTS_PER_MOTOR_REV)/
                (WHEEL_DIAMETER_INCHES*3.1415);
    }
}