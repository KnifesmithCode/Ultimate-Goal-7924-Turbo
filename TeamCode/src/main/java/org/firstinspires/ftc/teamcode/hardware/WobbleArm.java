package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.AccessoryPosition;

@Config
public class WobbleArm {
    private static final int TICKS_PER_REVOLUTION = 1680;

    // Arm values: 0.2 to 1.0
    // Claw values: 0.2 to 0.85

    public DcMotor armMotor;
    public Servo clawServo;

    private AccessoryPosition armPos;
    private AccessoryPosition clawPos;

    public WobbleArm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        clawServo = hardwareMap.get(Servo.class, "claw");

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawServo.setDirection(Servo.Direction.REVERSE);

        armPos = AccessoryPosition.UP;
        clawPos = AccessoryPosition.OPEN;
    }

    public AccessoryPosition toggleClaw() {
        // This if block could be simplified, but I think that it is more expressive this way
        if (clawPos == AccessoryPosition.CLOSED) {
            clawPos = AccessoryPosition.OPEN;
        } else if (clawPos == AccessoryPosition.OPEN) {
            clawPos = AccessoryPosition.CLOSED;
        } else {
            // This should not happen, but just in case, open the claw
            clawPos = AccessoryPosition.OPEN;
        }

        updateClawPosition();
        return clawPos;
    }

    public AccessoryPosition getArmPosition() {
        return armPos;
    }

    public AccessoryPosition getClawPosition() {
        return clawPos;
    }

    public void setArmPosition(AccessoryPosition pos) {
        this.armPos = pos;
        updateArmPosition();
    }

    public void setClawPosition(AccessoryPosition pos) {
        this.clawPos = pos;
        updateClawPosition();
    }

    private void updateArmPosition() {
        switch (armPos) {
            case DOWN:
                armMotor.setTargetPosition(-1530);
                break;
            case MIDDLE:
                armMotor.setTargetPosition(-640);
                break;
            default:
            case UP:
                armMotor.setTargetPosition(0);
                break;
        }
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8d);
    }

    private void updateClawPosition() {
        switch (clawPos) {
            case CLOSED:
                clawServo.setPosition(1.0d);
                break;
            default:
            case OPEN:
                clawServo.setPosition(0.4d);
                break;
        }
    }

    public void updatesPositions() {
        updateArmPosition();
        updateClawPosition();
    }
}
