package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrivetrain {
    DcMotorEx[] motors = new DcMotorEx[3];

    /**
     * Create and initialize the Mecanum drivetrain from the HardwareMap
     *
     * @param hardwareMap The HardwareMap given in the init() portion of the OpMode
     */
    public MecanumDrivetrain(HardwareMap hardwareMap) {
        // Initialize motors
        // Left motors are first, then right motors
        /*
            0 2
            1 3
         */
        motors[0] = hardwareMap.get(DcMotorEx.class, "lf");
        motors[1] = hardwareMap.get(DcMotorEx.class, "lr");
        motors[2] = hardwareMap.get(DcMotorEx.class, "rf");
        motors[3] = hardwareMap.get(DcMotorEx.class, "rr");

        // Set motor directions
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.FORWARD);
        motors[3].setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Send given power to each and every motor
     *
     * @param power An array of power doubles to send to the motors, in the same order as motors
     */
    public void setPowers(double[] power) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(power[i]);
        }
    }
}
