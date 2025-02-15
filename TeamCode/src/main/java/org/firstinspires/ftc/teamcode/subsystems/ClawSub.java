package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSub {

    public Servo clawServo;
    public CRServo leftRotate;
    public CRServo rightRotate;

    public void setRotate(double position) {
        leftRotate.setPower(position);
        rightRotate.setPower(position);
    }

    public void setClawServo(double position) {
        clawServo.setPosition(position);
    }
}
