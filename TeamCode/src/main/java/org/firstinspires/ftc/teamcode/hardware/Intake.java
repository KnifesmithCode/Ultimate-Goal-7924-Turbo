package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    public DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void runForward() {
        intakeMotor.setPower(1.0d);
    }

    public void runBackward() {
        intakeMotor.setPower(-1.0d);
    }

    public void stop() {
        intakeMotor.setPower(0.0d);
    }

}
