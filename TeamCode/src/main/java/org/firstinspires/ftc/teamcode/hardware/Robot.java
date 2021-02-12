package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.hardware.RRMecanumDrivetrain;

public class Robot {
    public RRMecanumDrivetrain dt;
    public RingLauncher launcher;
    public Intake intake;
    public WobbleArm wobbleArm;
    public WobbleLatch wobbleLatch;

    /**
     * Create and initialize the robot from the HardwareMap
     * Most of this section will likely be passing the HardwareMap to the children of the robot
     * to initialize them
     *
     * @param hardwareMap The HardwareMap given in the init() portion of the OpMode
     */
    public Robot(HardwareMap hardwareMap) {
        // Initialize the drivetrain
        dt = new RRMecanumDrivetrain(hardwareMap);

        // Initialize the launcher
        launcher = new RingLauncher(hardwareMap);

        // Initialize the intake
        intake = new Intake(hardwareMap);

        // Initialize the wobble goal arm
        wobbleArm = new WobbleArm(hardwareMap);

        // Initialize the wobble goal latch
        wobbleLatch = new WobbleLatch(hardwareMap);
    }
}
