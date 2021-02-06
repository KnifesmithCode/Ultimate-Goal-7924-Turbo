package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.ServoPosition;

@Config
@Autonomous(name = "BasicAuto", group = "Autonomous")
public class BasicAuto extends LinearOpMode {
    private ElapsedTime runtime;
    private Robot robot;

    public static double vmax = 0.6d;

    public static double path1 = 1000d;
    public static double path2 = 500d;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Resetting encoders...");
        telemetry.update();

        robot.dt.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dt.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (runtime.milliseconds() < path1 && opModeIsActive()) {
            double power = Math.min((runtime.milliseconds() / 800d) * vmax, vmax);
            robot.dt.setPowers(-power);
        }

        robot.launcher.updateVelocity();

        while (runtime.milliseconds() < path1 * 2d && opModeIsActive()) {
            double power = Math.min((((path1 * 2d) - 800d) / runtime.milliseconds()) * vmax, vmax);
            robot.dt.setPowers(-power);
        }

        robot.dt.setPowers(0d);

        for (int i = 0; i < 3; i++) {
            launchRing(runtime);
        }

        runtime.reset();

        while (runtime.milliseconds() < path2 && opModeIsActive()) {
            double power = Math.min((runtime.milliseconds() / 800d) * vmax, vmax);
            robot.dt.setPowers(-power);
        }

        while (runtime.milliseconds() < path2 * 2d && opModeIsActive()) {
            double power = Math.min((((path2 * 2d) - 800d) / runtime.milliseconds()) * vmax, vmax);
            robot.dt.setPowers(-power);
        }

        stop();
    }

    private void launchRing(ElapsedTime runtime) {
        double initial = runtime.milliseconds();
        while (runtime.milliseconds() < (initial + 1000d) && opModeIsActive()) {
            robot.launcher.setHammerPosition(ServoPosition.CLOSED);
        }
        while (runtime.milliseconds() < (initial + 2000d) && opModeIsActive()) {
            robot.launcher.setHammerPosition(ServoPosition.OPEN);
        }
    }
}
