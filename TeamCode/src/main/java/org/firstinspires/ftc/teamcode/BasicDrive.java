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

@TeleOp(name = "2025 Main Teleop")
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
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorSub.leftElevator = hardwareMap.dcMotor.get("leftElevator");
        elevatorSub.leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorSub.rightElevator = hardwareMap.get(DcMotor.class, "rightElevator");
        elevatorSub.rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorSub.rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorSub.lateralLeftElevator = hardwareMap.get(DcMotor.class, "lateralLeftElevator");
        elevatorSub.lateralLeftElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorSub.lateralRightElevator = hardwareMap.get(DcMotor.class, "lateralRightElevator");

        clawSub.leftRotate = hardwareMap.get(CRServo.class, "leftRotate");
        clawSub.rightRotate = hardwareMap.get(CRServo.class, "rightRotate");
        clawSub.clawServo = hardwareMap.get(Servo.class, "claw");

        intakeSub.intakeServo = hardwareMap.get(CRServo.class, "intake");
        intakeSub.intakeRotation = hardwareMap.get(Servo.class, "rotate");

        elevatorSub.leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorSub.rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorSub.leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorSub.rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            int leftPosition = elevatorSub.leftElevator.getCurrentPosition();
            int rightPosition = elevatorSub.leftElevator.getCurrentPosition();

            // Show the position of the motor on telemetry
            telemetry.addData("Left Encoder Position", leftPosition);
            telemetry.addData("Right Encoder Position", rightPosition);
            telemetry.update();

            if (gamepad1.left_bumper) {
                elevatorSub.setPower(0.75);
                telemetry.update();
            } else if (gamepad1.right_bumper) {
                elevatorSub.setPower(-0.75);
            } else {
                elevatorSub.setPower(0);
            }

            if (gamepad1.x) {
                elevatorSub.setLateralPower(0.75);
            } else if (gamepad1.y) {
                elevatorSub.setLateralPower(-0.75);
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

            if (gamepad2.a) {
                clawSub.setClawServo(0.75);
            } else if (gamepad2.b) {
                clawSub.setClawServo(1);
            }

            if (gamepad2.x) {
                intakeSub.setRotate(1);
            } else if (gamepad2.y) {
                intakeSub.setRotate(0);
            }

            if (gamepad2.left_bumper) {
                clawSub.setRotate(-0.6);
            } else if (gamepad2.right_bumper) {
                clawSub.setRotate(0.6);
            } else {
                clawSub.setRotate(0);
            }

            if (gamepad2.back) {
                elevatorSub.lateralLeftElevator.setPower(0.5);
            } else {
                elevatorSub.lateralLeftElevator.setPower(0);
            }

            if (gamepad2.start) {
                elevatorSub.lateralRightElevator.setPower(0.5);
            } else {
                elevatorSub.lateralRightElevator.setPower(0);
            }


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;

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
