package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleLatch {
    Servo latchServo;

    private ServoPosition latchPos;

    public WobbleLatch(HardwareMap hardwareMap) {
        latchPos = ServoPosition.CLOSED;
        latchServo = hardwareMap.get(Servo.class, "latch");
    }

    public ServoPosition toggleLatch() {
        // This if block could be simplified, but I think that it is more expressive this way
        if(latchPos == ServoPosition.CLOSED) {
            latchPos = ServoPosition.OPEN;
        } else if (latchPos == ServoPosition.OPEN) {
            latchPos = ServoPosition.CLOSED;
        } else {
            // This should not happen, but just in case, open the claw
            latchPos = ServoPosition.OPEN;
        }

        updateLatchPosition();
        return latchPos;
    }

    public void setLatchPosition(ServoPosition pos) {
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
