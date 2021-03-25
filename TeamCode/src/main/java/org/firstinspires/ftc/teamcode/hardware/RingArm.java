package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.AccessoryPosition;

import static org.firstinspires.ftc.teamcode.util.AccessoryPosition.*;

@Config
public class RingArm {
    public static double UP_POS = 0.5d;
    public static double DOWN_POS = 0.08d;

    public Servo ringArmServo;
    
    private AccessoryPosition ringArmPos;
    
    public RingArm(HardwareMap hardwareMap) {
        ringArmPos = UP;
        ringArmServo = hardwareMap.get(Servo.class, "ringarm");
    }

    public void setRingArmPosition(AccessoryPosition pos) {
        ringArmPos = pos;
        updateRingArmPosition();
    }

    private void updateRingArmPosition() {
        switch(ringArmPos) {
            case UP:
            default:
                ringArmServo.setPosition(UP_POS);
                break;
            case DOWN:
                ringArmServo.setPosition(DOWN_POS);
                break;
        }
    }
}
