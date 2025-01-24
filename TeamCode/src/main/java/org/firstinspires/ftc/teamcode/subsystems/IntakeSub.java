package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSub {
    public CRServo intakeServo;
    public Servo intakeRotation;

    public void setIntake(double power) {
        intakeServo.setPower(power);
    }

    public void setRotate(double position) {
        intakeRotation.setPosition(position);
    }
}
