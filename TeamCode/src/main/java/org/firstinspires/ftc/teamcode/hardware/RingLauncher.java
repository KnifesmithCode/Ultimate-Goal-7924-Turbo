package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.AccessoryPosition;

@Config
public class RingLauncher {
    public static PIDFCoefficients LAUNCHER_PID = new PIDFCoefficients();

    // Static members to be used by the Dashboard
    public static double LAUNCHER_VELOCITY = 0.85d;
    public static double POWERSHOT_VELOCITY = 0.76d;
    public static double HAMMER_POS = 0.0d;

    public enum Target {
        TOWER_TELEOP, POWERSHOT
    }

    // FOR POWERSHOTS, a speed of approximately 1680 is good (0.84d)

    private static double COUNTS_PER_REVOLUTION = 145.6d;
    private static double MAX_RPS = 1150d / 60;

    private static double approx_vmax = 2000d;

    public DcMotorEx flywheelMotor;
    public Servo hammerServo;
    // Hammer positions:
    // 0.0 for open (not pushed in/resting)
    // 0.45 for closed (pushed in/launched)

    private Target currentTarget;

    /**
     * Create and initialize the ring launcher from the HardwareMap
     *
     * @param hardwareMap The HardwareMap given in the init() portion of the OpMode
     */
    public RingLauncher(HardwareMap hardwareMap) {
        // Initialize components of the ring launcher
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        hammerServo = hardwareMap.get(Servo.class, "hammer");

        // Set the flywheel to run using encoder, important for keeping velocity as the battery wavers
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LAUNCHER_PID = flywheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set PIDF coefficients for the flywheel motor
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PID);

        // Set the flywheel's direction (positive velocity should shoot rings)
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set the current target to the Tower by default
        currentTarget = Target.TOWER_TELEOP;
    }

    public void setLauncherVelocity(double newVelocity) {
        RingLauncher.LAUNCHER_VELOCITY = newVelocity;
    }

    /**
     * Update the launcher's velocity
     */
    public void updateVelocity() {
        switch(currentTarget) {
            case TOWER_TELEOP:
            default:
                flywheelMotor.setVelocity(approx_vmax * LAUNCHER_VELOCITY);
                break;
            case POWERSHOT:
                flywheelMotor.setVelocity(approx_vmax * POWERSHOT_VELOCITY);
        }
    }

    public double getTargetV() {
        switch(currentTarget) {
            default:
            case TOWER_TELEOP:
                return approx_vmax * LAUNCHER_VELOCITY;
            case POWERSHOT:
                return approx_vmax * POWERSHOT_VELOCITY;
        }
    }

    /**
     * Update the hammer's position
     */
    public void updateHammer() {
        hammerServo.setPosition(HAMMER_POS);
    }


    /**
     * Set the target which the launcher will use in order to set velocity
     * @param newTarget The new target to aim for (an enum of type {@link Target})
     */
    public void setTarget(Target newTarget) {
        this.currentTarget = newTarget;
    }

    public String getTarget() {
        return currentTarget.name();
    }

    /**
     * Does what it says on the tin
     *
     * @param pos An enum for position (either HammerPosition.ENGAGED or HammerPosition.DISENGAGED)
     */
    public void setHammerPosition(AccessoryPosition pos) {
        switch (pos) {
            case ENGAGED:
                hammerServo.setPosition(0.45d);
                break;
            case DISENGAGED:
            default:
                hammerServo.setPosition(0.15d);
        }
    }

    public boolean isAtTargetVelocity() {
        return targetVelocityDelta() <= 20;
    }

    public double targetVelocityDelta() {
        return Math.abs(getTargetV() - flywheelMotor.getVelocity());
    }

    public AccessoryPosition getHammerPosition() {
        return hammerServo.getPosition() == 0.65d ? AccessoryPosition.CLOSED : AccessoryPosition.OPEN;
    }
}
