package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public MecanumDrivetrain dt;

    /**
     * Create and initialize the robot from the HardwareMap
     * Most of this section will likely be passing the HardwareMap to the children of the robot
     * to initialize them
     *
     * @param hardwareMap The HardwareMap given in the init() portion of the OpMode
     */
    public Robot(HardwareMap hardwareMap) {
        // Initialize the drivetrain
        dt = new MecanumDrivetrain(hardwareMap);
    }
}
