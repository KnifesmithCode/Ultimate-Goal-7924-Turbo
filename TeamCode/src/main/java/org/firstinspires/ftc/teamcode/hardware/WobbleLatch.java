package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.AccessoryPosition;

public class WobbleLatch {
    Servo latchServo;

    private AccessoryPosition latchPos;

    public WobbleLatch(HardwareMap hardwareMap) {
        latchPos = AccessoryPosition.CLOSED;
        latchServo = hardwareMap.get(Servo.class, "latch");
    }

    public AccessoryPosition toggleLatch() {
        // This if block could be simplified, but I think that it is more expressive this way
        if(latchPos == AccessoryPosition.CLOSED) {
            latchPos = AccessoryPosition.OPEN;
        } else if (latchPos == AccessoryPosition.OPEN) {
            latchPos = AccessoryPosition.CLOSED;
        } else {
            // This should not happen, but just in case, open the claw
            latchPos = AccessoryPosition.OPEN;
        }

        updateLatchPosition();
        return latchPos;
    }

    public void setLatchPosition(AccessoryPosition pos) {
        this.latchPos = pos;
        updateLatchPosition();
    }

    private void updateLatchPosition() {
        switch(latchPos) {
            case CLOSED:
                latchServo.setPosition(1.0d);
                break;
            case OPEN:
            default:
                latchServo.setPosition(0.2d);
                break;
        }
    }
}
