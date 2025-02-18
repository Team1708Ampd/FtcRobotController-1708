package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ElevatorSub {
    public DcMotor leftElevator;
    public DcMotor rightElevator;
    public DcMotor lateralLeftElevator;
    public DcMotor lateralRightElevator;

    public void setPower(double power) {
        leftElevator.setPower(power);
        rightElevator.setPower(power);
    }

    public void setLateralPower(double power) {
        lateralLeftElevator.setPower(power);
        lateralRightElevator.setPower(power);
    }
}
