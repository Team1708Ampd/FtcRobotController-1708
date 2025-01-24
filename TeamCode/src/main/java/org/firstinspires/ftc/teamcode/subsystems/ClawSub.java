package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawSub {

    public Servo clawServo;
    public Servo leftRotate;
    public Servo rightRotate;

    public void setRotate(double position) {
        leftRotate.setPosition(position);
        rightRotate.setPosition(position);
    }

    public void setClawServo(double position) {
        clawServo.setPosition(position);
    }
}
