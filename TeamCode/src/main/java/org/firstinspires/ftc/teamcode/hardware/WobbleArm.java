package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class WobbleArm {
    public static double ARM_POS = 0.0d;
    public static double CLAW_POS = 0.0d;

    // Arm values: 0.2 to 1.0
    // Claw values: 0.2 to 0.85

    Servo armServo;
    Servo clawServo;

    private ServoPosition armPos;
    private ServoPosition clawPos;

    public WobbleArm(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, "arm");
        clawServo = hardwareMap.get(Servo.class, "claw");
    }

    public ServoPosition toggleClaw() {
        // This if block could be simplified, but I think that it is more expressive this way
        if(clawPos == ServoPosition.CLOSED) {
            clawPos = ServoPosition.OPEN;
        } else if (clawPos == ServoPosition.OPEN) {
            clawPos = ServoPosition.CLOSED;
        } else {
            // This should not happen, but just in case, open the claw
            clawPos = ServoPosition.OPEN;
        }

        updateClawPosition();
        return clawPos;
    }

    public ServoPosition getArmPosition() {
        return armPos;
    }

    public ServoPosition getClawPosition() {
        return clawPos;
    }

    public void setArmPosition(ServoPosition pos) {
        this.armPos = pos;
        updateArmPosition();
    }

    public void setClawPosition(ServoPosition pos) {
        this.clawPos = pos;
        updateClawPosition();
    }

    public void updateArmPosition() {
        switch (armPos) {
            case DOWN:
                armServo.setPosition(0.9d);
                break;
            case MIDDLE:
                armServo.setPosition(0.60d);
                break;
            default:
            case UP:
                armServo.setPosition(0.2d);
                break;
        }
    }

    public void updateClawPosition() {
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
