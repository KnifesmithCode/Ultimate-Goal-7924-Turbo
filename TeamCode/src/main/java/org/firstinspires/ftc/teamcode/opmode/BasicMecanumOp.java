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

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RingLauncher;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.ServoPosition;
import org.firstinspires.ftc.teamcode.util.NumberUtil;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode lista
 */

@TeleOp(name = "Basic Mecanum OpMode", group = "TeleOp")
// @Disabled
@Config
public class BasicMecanumOp extends OpMode {
    // Allow changing of mecanum x-axis coefficient
    public static double MECANUM_X_OFFSET = 1.5d;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    Robot robot;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    ServoPosition armPos = ServoPosition.DOWN;
    boolean claw = false;
    double lastClaw = 0d;

    Gamepad oldGamepad1 = new Gamepad();
    Gamepad oldGamepad2 = new Gamepad();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot = new Robot(hardwareMap);

        robot.launcher.setHammerPosition(ServoPosition.OPEN);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * MECANUM_X_OFFSET;
        double rot = gamepad1.right_stick_x;

        double[] motorPower = new double[4];

        motorPower[0] = y + x + rot;
        motorPower[1] = y - x + rot;
        motorPower[2] = y - x - rot;
        motorPower[3] = y + x - rot;

        double max = NumberUtil.max(motorPower);

        if (max > 1) {
            for (int i = 0; i < motorPower.length; i++) {
                motorPower[i] /= max;
            }
        }

        if(gamepad1.x) {
            for(int i = 0; i < motorPower.length; i++) {
                motorPower[i] *= -1.0d;
            }
        }

        robot.dt.setPowers(motorPower);
        robot.intake.updatePower();

        if(gamepad1.dpad_up && !oldGamepad1.dpad_up) {
            robot.launcher.setLauncherVelocity(Math.min(RingLauncher.LAUNCHER_VELOCITY + 0.01d, 1.0d));
        }
        if(gamepad1.dpad_down && !oldGamepad1.dpad_down) {
            robot.launcher.setLauncherVelocity(Math.max(RingLauncher.LAUNCHER_VELOCITY - 0.01d, 0.0d));
        }

        robot.launcher.updateVelocity();

        double v = robot.launcher.flywheelMotor.getVelocity();
        double delta = Math.abs(robot.launcher.getTargetV() - v);

        if (gamepad1.a && delta <= 20) {
            robot.launcher.setHammerPosition(ServoPosition.CLOSED);
        } else {
            robot.launcher.setHammerPosition(ServoPosition.OPEN);
        }

        if(gamepad1.right_bumper && !oldGamepad1.right_bumper) {
            robot.wobbleArm.toggleClaw();
        }

        armPos = ServoPosition.MIDDLE;
        if(gamepad1.b) armPos = ServoPosition.DOWN;
        if(gamepad1.y) armPos = ServoPosition.MIDDLE;
        if(gamepad1.left_bumper) armPos = ServoPosition.UP;

        robot.wobbleArm.setArmPosition(armPos);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "lf (%.2f) | rf (%.2f)", motorPower[0], motorPower[2]);
        telemetry.addData("Motors", "lr (%.2f) | rr (%.2f)", motorPower[1], motorPower[3]);
        telemetry.addData("LauncherV", "%.3f",
                v);
        dashboardTelemetry.addData("LauncherV", "%.3f",
                v);

        try {
            oldGamepad1.copy(gamepad1);
            oldGamepad2.copy(gamepad2);
        } catch (RobotCoreException e) {
            telemetry.addData("GP ERROR", "Unable to copy gamepads");
        }
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.launcher.setHammerPosition(ServoPosition.OPEN);
    }

}
