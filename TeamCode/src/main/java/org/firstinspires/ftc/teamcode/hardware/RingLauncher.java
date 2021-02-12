package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.AccessoryPosition;

@Config
public class RingLauncher {
    // Static members to be used by the Dashboard
    public static double LAUNCHER_VELOCITY = 0.90d;
    public static double HAMMER_POS = 0.0d;

    // FOR POWERSHOTS, a speed of approximately 1680 is good (0.84d)

    private static double COUNTS_PER_REVOLUTION = 145.6d;
    private static double MAX_RPS = 1150d / 60;

    private static double approx_vmax = 2000d;

    public DcMotorEx flywheelMotor;
    public Servo hammerServo;
    // Hammer positions:
    // 0.0 for open (not pushed in/resting)
    // 0.45 for closed (pushed in/launched)

    /**
     * Create and initialize the ring launcher from the HardwareMap
     *
     * @param hardwareMap The HardwareMap given in the init() portion of the OpMode
     */
    public RingLauncher(HardwareMap hardwareMap) {
        LAUNCHER_VELOCITY = 0.89d;

        // Initialize components of the ring launcher
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        hammerServo = hardwareMap.get(Servo.class, "hammer");

        // Set the flywheel to run using encoder, important for keeping velocity as the battery wavers
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the flywheel's direction (positive velocity should shoot rings)
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setLauncherVelocity(double newVelocity) {
        RingLauncher.LAUNCHER_VELOCITY = newVelocity;
    }

    /**
     * Update the launcher's velocity
     */
    public void updateVelocity() {
        /*flywheelMotor.setVelocity(
                COUNTS_PER_REVOLUTION * MAX_RPM * LAUNCHER_VELOCITY
        );*/
        //flywheelMotor.setPower(LAUNCHER_VELOCITY);
        flywheelMotor.setVelocity(approx_vmax * LAUNCHER_VELOCITY);
    }

    public double getTargetV() {
        return approx_vmax * LAUNCHER_VELOCITY;
    }

    /**
     * Update the hammer's position
     */
    public void updateHammer() {
        hammerServo.setPosition(HAMMER_POS);
    }

    /**
     * Does what it says on the tin (open is not shooting; closed is shooting)
     *
     * @param pos An enum for position (either HammerPosition.OPEN or HammerPosition.CLOSED)
     */
    public void setHammerPosition(AccessoryPosition pos) {
        switch (pos) {
            case ENGAGED:
                hammerServo.setPosition(0.45d);
                break;
            case DISENGAGED:
            default:
                hammerServo.setPosition(0.1d);
        }
    }

    public boolean isAtTargetVelocity() {
        return  Math.abs(getTargetV() - flywheelMotor.getVelocity()) <= 20;
    }

    public AccessoryPosition getHammerPosition() {
        return hammerServo.getPosition() == 0.65d ? AccessoryPosition.CLOSED : AccessoryPosition.OPEN;
    }
}
