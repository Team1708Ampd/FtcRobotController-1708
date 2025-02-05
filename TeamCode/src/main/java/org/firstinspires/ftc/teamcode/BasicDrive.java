package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ClawSub;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSub;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSub;

@TeleOp(name = "BasicDrive")
public class BasicDrive extends LinearOpMode {

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    ElevatorSub elevatorSub = new ElevatorSub();
    ClawSub clawSub = new ClawSub();
    IntakeSub intakeSub = new IntakeSub();

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "rightFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "rightBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorSub.leftElevator = hardwareMap.get(DcMotor.class, "leftElevator");
        elevatorSub.leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorSub.rightElevator = hardwareMap.get(DcMotor.class, "rightElevator");
        elevatorSub.rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorSub.rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorSub.lateralLeftElevator = hardwareMap.get(DcMotor.class, "lateralLeftElevator");
        elevatorSub.lateralLeftElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorSub.lateralRightElevator = hardwareMap.get(DcMotor.class, "lateralRightElevator");

        clawSub.leftRotate = hardwareMap.get(Servo.class, "leftRotate");
        clawSub.rightRotate = hardwareMap.get(Servo.class, "rightRotate");
        clawSub.clawServo = hardwareMap.get(Servo.class, "claw");

        intakeSub.intakeServo = hardwareMap.get(CRServo.class, "intake");
        intakeSub.intakeRotation = hardwareMap.get(Servo.class, "rotate");
        intakeSub.intakeRotation.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                elevatorSub.setPower(0.5);
            } else if (gamepad1.right_bumper) {
                elevatorSub.setPower(-0.5);
            } else {
                elevatorSub.setPower(0);
            }

            if (gamepad1.x) {
                elevatorSub.setLateralPower(0.5);
            } else if (gamepad1.y) {
                elevatorSub.setLateralPower(-0.5);
            } else {
                elevatorSub.setLateralPower(0);
            }

            if (gamepad1.a) {
                intakeSub.setIntake(-1);
            } else if (gamepad1.b) {
                intakeSub.setIntake(1);
            }else {
                intakeSub.setIntake(0);
            }

            if (gamepad1.back) {
                intakeSub.intakeRotation.setPosition(0);
            } else if (gamepad1.start) {
                intakeSub.intakeRotation.setPosition(0.3);
            }


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

        }
    }
}
