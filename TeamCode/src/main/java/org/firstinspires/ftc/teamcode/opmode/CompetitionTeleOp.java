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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.RingLauncher;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.AccessoryPosition;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;

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
    private enum State {
        DRIVER_CONTROL, AUTO_AIMING, AUTO_POWERSHOT
    }

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    Robot robot;

    double lastShot = 0;

    State currentState;

    Pose2d poseEstimate;

    AccessoryPosition armPos = AccessoryPosition.MIDDLE;

    FtcDashboard dashboard;

    ArrayList<Trajectory> powershotTrajectories = new ArrayList<>();
    private int powershotStep = 0;
    private double powershotTime = 0d;

    private double lastJam = 0d;

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

        powershotTrajectories.add(
                robot.dt.trajectoryBuilder(PoseStorage.POWERSHOT_POSE, PoseStorage.POWERSHOT_POSE.getHeading())
                        .strafeTo(new Vector2d(0.0D, -2.0D)).build());
        powershotTrajectories.add(
                robot.dt.trajectoryBuilder(powershotTrajectories.get(0).end(), Math.toRadians(powershotTrajectories.get(0).end().getHeading()))
                        .strafeTo(new Vector2d(0.0D, 5.5D)).build());
        powershotTrajectories.add(
                robot.dt.trajectoryBuilder(powershotTrajectories.get(1).end(), Math.toRadians(powershotTrajectories.get(1).end().getHeading()))
                        .strafeTo(new Vector2d(0.0D, 13.0D)).build());

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
        switch (currentState) {
            case DRIVER_CONTROL:
            default:
                //#region Drivetrain
                // Send power to drivetrain (using Roadrunner)
                double speedMultiplier = 1 - (gamepad1.left_trigger * 0.7);
                robot.dt.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speedMultiplier,
                                -gamepad1.left_stick_x * speedMultiplier,
                                -gamepad1.right_stick_x * speedMultiplier
                        )
                );
                //#endregion

                //#region Launcher
                // Allow adjustment of the launcher velocity based on user feedback
                if (gamepad1.dpad_up && !oldGamepad1.dpad_up) {
                    robot.launcher.setLauncherVelocity(Math.min(RingLauncher.LAUNCHER_VELOCITY + 0.01d, 1.0d));
                }
                if (gamepad1.dpad_down && !oldGamepad1.dpad_down) {
                    robot.launcher.setLauncherVelocity(Math.max(RingLauncher.LAUNCHER_VELOCITY - 0.01d, 0.0d));
                }

                // Allow the usage of gamepad2 to change the target of the launcher
                if (gamepad2.a) {
                    robot.launcher.setTarget(RingLauncher.Target.TOWER_TELEOP);
                } else if (gamepad2.b) {
                    robot.launcher.setTarget(RingLauncher.Target.POWERSHOT);
                }

                robot.launcher.updateVelocity();

                // Launch rings if the launcher is at target velocity
                if (gamepad1.a) {
                    if (robot.launcher.isAtTargetVelocity()) {
                        robot.launcher.setHammerPosition(AccessoryPosition.ENGAGED);
                        lastShot = runtime.milliseconds();
                    }
                } else {
                    robot.launcher.setHammerPosition(AccessoryPosition.DISENGAGED);
                }

                // Store velocity values to report back to telemetry
                targetV = robot.launcher.getTargetV();
                //#endregion

                //#region Intake
                /* Run the intake backward if the back button is pressed
                   This is useful for stuck rings, or for cases of carrying four rings
                   In addition, if the motor is drawing over seven amps (a fair number)
                   then reverse the motor, because it must be jammed.
                 */
                if (!gamepad1.back && robot.intake.intakeMotor.getCurrent(CurrentUnit.AMPS) < 7.0d) {
                    if (robot.intake.intakeMotor.getCurrent(CurrentUnit.AMPS) > 7.0d) {
                        lastJam = runtime.milliseconds();
                    }
                    if (runtime.milliseconds() - lastJam < 250d) {
                        robot.intake.runBackward();
                    } else {
                        robot.intake.runForward();
                    }
                } else {
                    robot.intake.runBackward();
                }
                //#endregion

                //#region Wobble Arm
                // Open or close the claw with the right bumper
                if (gamepad1.right_bumper && !oldGamepad1.right_bumper) {
                    robot.wobbleArm.toggleClaw();
                }

                // Set the wobble arm position based on where it is currently (null-safe)
                armPos =
                        robot.wobbleArm != null && robot.wobbleArm.getArmPosition() != null ?
                                robot.wobbleArm.getArmPosition() :
                                AccessoryPosition.MIDDLE;
                // User control of the wobble arm position
                if (gamepad1.b) {
                    armPos = AccessoryPosition.DOWN;
                } else if (gamepad1.y) {
                    armPos = AccessoryPosition.MIDDLE;
                } else if (gamepad1.left_bumper) {
                    armPos = AccessoryPosition.UP;
                }

                // While I cannot imagine wobbleArm being null, Android Studio certainly can
                // Check to make sure that wobbleArm has been initialized
                if (robot.wobbleArm != null) robot.wobbleArm.setArmPosition(armPos);
                //#endregion

                //#region Ring Arm
                if(gamepad2.dpad_up) {
                    robot.ringArm.setRingArmPosition(AccessoryPosition.UP);
                } else if (gamepad2.dpad_down) {
                    robot.ringArm.setRingArmPosition(AccessoryPosition.DOWN);
                }
                //#endregion

                //#region Macros & Misc.
                // Automatic shooting positioning macro
                if (gamepad1.dpad_right) {
                    // Calculate a trajectory to the shooting pose
                    // TODO: Do this on another thread for slight speed increases
                    Trajectory traj = robot.dt.trajectoryBuilder(poseEstimate)
                            .splineToLinearHeading(
                                    PoseStorage.SHOOTING_POSE,
                                    Math.toRadians(180.0))
                            .build();

                    // Follow the given trajectory
                    robot.dt.followTrajectoryAsync(traj);

                    currentState = State.AUTO_AIMING;
                }

                // Autmatic powershot macro
                if (gamepad2.y) {
                    // Start from powershot step zero
                    powershotStep = 0;

                    // Update the launcher to target powershots
                    robot.launcher.setTarget(RingLauncher.Target.POWERSHOT);
                    robot.launcher.updateVelocity();

                    // Set the PoseEstimate to a constant (POWERSHOT_POSE; along the east wall)
                    robot.dt.setPoseEstimate(PoseStorage.POWERSHOT_POSE);
                    // Begin following the trajectory
                    robot.dt.followTrajectoryAsync(powershotTrajectories.get(powershotStep));

                    currentState = State.AUTO_POWERSHOT;
                }

                // Reset the robot's current position to the shooting pose
                // Used when the robot makes a shot and
                if (gamepad2.x) {
                    robot.dt.setPoseEstimate(PoseStorage.SHOOTING_POSE);
                }
                //#endregion
                break;
            case AUTO_AIMING:
                if (robot.dt.isBusy()) {
                    telemetry.addData("Auto-Aim", "Pathing to shooting position...");
                    // Run the intake backward while automatically aiming so that we do not intake a fourth ring
                    robot.intake.runBackward();
                } else {
                    // Once the robot is done, return control to the driver
                    currentState = State.DRIVER_CONTROL;
                }
                break;
            case AUTO_POWERSHOT:
                if (robot.dt.isBusy()) {
                    telemetry.addData("Auto-PS", "Pathing to powershots");
                } else {
                    // Check if it has been enough time since a ring was launched
                    if (powershotTime == 0d) {
                        // Launch a ring and set the last ring launch time
                        robot.launcher.setHammerPosition(AccessoryPosition.ENGAGED);
                        powershotTime = runtime.milliseconds();
                    } else {
                        // Wait for 0.75s so that a ring launches properly
                        // If it has been enough time, go to the next powershot
                        if (runtime.milliseconds() - powershotTime > 750) {
                            robot.launcher.setHammerPosition(AccessoryPosition.DISENGAGED);
                            if (powershotStep < 2) {
                                // Go to the next powershot
                                powershotTime = 0d;
                                powershotStep++;
                                robot.dt.followTrajectoryAsync(powershotTrajectories.get(powershotStep));
                            } else {
                                // Reset the powershot status
                                powershotTime = 0;
                                // Return control to the driver
                                robot.launcher.setTarget(RingLauncher.Target.TOWER_TELEOP);
                                currentState = State.DRIVER_CONTROL;
                            }
                        }
                    }
                }
                break;
        }

        // Update the drivetrain for the robot
        robot.dt.update();
        // Get the Pose estimate, which is created by the odometry pods
        poseEstimate = robot.dt.getPoseEstimate();

        //#region Driver Station telemetry
        telemetry.addData("State", currentState.name());
        telemetry.addData("TargetV", targetV);
        telemetry.addData("LauncherV", v);
        telemetry.addData("Target", robot.launcher.getTarget());
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
        PoseStorage.POSE = robot.dt.getPoseEstimate();
    }

}
