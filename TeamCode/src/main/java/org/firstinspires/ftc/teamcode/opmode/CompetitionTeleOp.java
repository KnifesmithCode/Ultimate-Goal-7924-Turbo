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
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RingLauncher;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.AccessoryPosition;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

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

@TeleOp(name = "Competition TeleOP", group = "TeleOp")
@Config
public class CompetitionTeleOp extends OpMode {
    public static PIDFCoefficients LAUNCHER_PID = new PIDFCoefficients();

    private enum State {
        DRIVER_CONTROL, AUTO_AIMING, AUTO_POWERSHOT
    }

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    Robot robot;

    State currentState;

    Pose2d poseEstimate;

    AccessoryPosition armPos = AccessoryPosition.MIDDLE;

    FtcDashboard dashboard;
    TelemetryPacket packet;
    Canvas canvas;

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

        robot.launcher.setHammerPosition(AccessoryPosition.OPEN);

        dashboard = FtcDashboard.getInstance();

        robot.dt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.dt.setPoseEstimate(PoseStorage.POSE);
        currentState = State.DRIVER_CONTROL;

        LAUNCHER_PID = robot.launcher.flywheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

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
        double targetV = 0;
        double v = 0;
        switch(currentState) {
            case DRIVER_CONTROL:
            default:
                //#region Drivetrain
                // Send power to drivetrain (using Roadrunner)
                robot.dt.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                // Update the pose information for the robot, based on the odometry pods
                robot.dt.updatePoseEstimate();
                poseEstimate = robot.dt.getPoseEstimate();
                //#endregion

                //#region Launcher
                // Allow adjustment of the launcher velocity based on user feedback
                if (gamepad1.dpad_up && !oldGamepad1.dpad_up) {
                    robot.launcher.setLauncherVelocity(Math.min(RingLauncher.LAUNCHER_VELOCITY + 0.01d, 1.0d));
                }
                if (gamepad1.dpad_down && !oldGamepad1.dpad_down) {
                    robot.launcher.setLauncherVelocity(Math.max(RingLauncher.LAUNCHER_VELOCITY - 0.01d, 0.0d));
                }

                robot.launcher.updateVelocity();

                // Launch rings if the launcher is at target velocity
                if (gamepad1.a && robot.launcher.isAtTargetVelocity()) {
                    robot.launcher.setHammerPosition(AccessoryPosition.ENGAGED);
                } else {
                    robot.launcher.setHammerPosition(AccessoryPosition.DISENGAGED);
                }

                // Store velocity values to report back to telemetry
                v = robot.launcher.flywheelMotor.getVelocity();
                targetV = robot.launcher.getTargetV();

                // Set PIDF coefficients for the flywheel motor
                robot.launcher.flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PID);
                //#endregion

                //#region Intake
                // Run the intake backward if the back button is pressed
                // This is useful for stuck rings, or for cases of carrying four rings
                if (!gamepad1.back) {
                    robot.intake.runForward();
                } else {
                    robot.intake.runBackward();
                }
                //#endregion

                //#region Wobble Arm
                // Open or close the claw with the right bumper
                if (gamepad1.right_bumper && !oldGamepad1.right_bumper) {
                    robot.wobbleArm.toggleClaw();
                }

                armPos =
                        robot.wobbleArm != null && robot.wobbleArm.getArmPosition() != null ?
                                robot.wobbleArm.getArmPosition() :
                                AccessoryPosition.MIDDLE;
                if (gamepad1.b) {
                    armPos = AccessoryPosition.DOWN;
                } else if (gamepad1.y) {
                    armPos = AccessoryPosition.MIDDLE;
                } else if (gamepad1.left_bumper) {
                    armPos = AccessoryPosition.UP;
                }

                // While I cannot imagine wobbleArm being null, Android Studio certainly can
                // Check to make sure that wobbleArm has been initialized
                if(robot.wobbleArm != null) robot.wobbleArm.setArmPosition(armPos);
                //#endregion
                break;
            case AUTO_AIMING:
                //TODO: Automatically aim the robot when requested
                break;
            case AUTO_POWERSHOT:
                //TODO: Automatically hit the powershots when requested
                break;
        }

        //#region Driver Station telemetry
        telemetry.addData("State", currentState.name());
        telemetry.addData("TargetV", targetV);
        telemetry.addData("LauncherV", v);
        //#endregion

        //#region Dashboard telemetry
        packet = new TelemetryPacket();

        packet.put("State", currentState.name());
        packet.put("x", poseEstimate.getX());
        packet.put("y", poseEstimate.getY());
        packet.put("heading", poseEstimate.getHeading());
        packet.put("TargetV", String.valueOf(Math.round(targetV)));
        packet.put("LauncherV", String.valueOf(Math.round(v)));

        // Allow the dashboard to show the robot position
        canvas = packet.fieldOverlay();
        DashboardUtil.drawRobot(canvas, poseEstimate);

        dashboard.sendTelemetryPacket(packet);
        //#endregion

        // Used to detect a button press vs a button hold
        try {
            oldGamepad1.copy(gamepad1);
            oldGamepad2.copy(gamepad2);
        } catch (RobotCoreException e) {
            telemetry.addData("GP ERROR", "Unable to copy gamepads");
        }

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.launcher.setHammerPosition(AccessoryPosition.OPEN);
    }

}
