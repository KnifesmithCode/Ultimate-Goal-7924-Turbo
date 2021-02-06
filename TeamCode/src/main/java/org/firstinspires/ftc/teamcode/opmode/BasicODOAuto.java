package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name="Basic ODO", group="Autonomous")
public class BasicODOAuto extends LinearOpMode {
    // ODO Wheel Diameter
    private static final double odoDiameter = 35.0d / 25.4d;
    // ODO Wheel ticks per revolution

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        for(DcMotorEx motor : robot.dt.motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        waitForStart();

        while(opModeIsActive()) {

        }
    }

    private void moveForward(double inches) {

    }
}
