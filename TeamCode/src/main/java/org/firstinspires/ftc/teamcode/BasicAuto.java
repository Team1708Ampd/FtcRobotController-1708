package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSub;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSub;
import org.firstinspires.ftc.teamcode.subsystems.ClawSub;
@Config
@Autonomous(name = "BLUE_AUTO_1")
public class BasicAuto extends LinearOpMode {

    // SUBSYSTEM MODULES
    // Mecanum drive
    MecanumDrive _drive;
    // Elevator
    ElevatorSub elevatorSub;
    // Intake
    IntakeSub intakeSub;
    // Claw
    ClawSub clawSub;

    // Telemetry objects
    TelemetryPacket _packet;

    public final double CLAW_SCORING_POS = 200;

    public void runOpMode() throws InterruptedException {

        elevatorSub = new ElevatorSub();
        intakeSub = new IntakeSub();
        clawSub = new ClawSub();

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

        _packet = new TelemetryPacket();

        // Set up the initial pose
        Pose2d initPose = new Pose2d(AutoConstants.START_POSE_X,
                                     AutoConstants.START_POSE_Y,
                                     AutoConstants.START_POSE_THETA);

        // Init Subsystem Modules
        _drive = new MecanumDrive(hardwareMap, initPose);


        // Init Vision

        // TRAJECTORIES
        // Create trajectories for all 3 starting positions
        // Note - A single trajectory will be defined for each starting configuration
        // and Pose Mapping will be utilized to translate these trajectories between
        // the Red and Blue Alliance sides

        // Start 1
        TrajectoryActionBuilder auto1 = _drive.actionBuilder(initPose)
                .splineTo(new Vector2d(40, 40), Math.toRadians(270))
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(53, 53));

        TrajectoryActionBuilder auto1_seg1 = _drive.actionBuilder(initPose)
                .strafeTo(new Vector2d(0, 35));

        TrajectoryActionBuilder auto1_seg2 = _drive.actionBuilder(initPose)
                .turnTo(Math.toRadians(180))
                .splineTo(new Vector2d(-45, 20), Math.toRadians(180))
                .strafeTo(new Vector2d(-45, 55));

        // Start 2
        TrajectoryActionBuilder auto2 = _drive.actionBuilder(initPose);

        // Start 3
        TrajectoryActionBuilder auto3 = _drive.actionBuilder(initPose);

        // Initialization routine
        while (!isStopRequested() && !opModeIsActive())
        {

        }

        // Safeguard
        if (isStopRequested()) return;

        // Chose the trajectory to run
        Action chosenTrajectory;

        // Run all actions
        Actions.runBlocking(new SequentialAction(
                auto1_seg1.build(),
                elevatorSub.RunElevator_Timed(0.2, 2),
                clawSub.SetClaw(CLAW_SCORING_POS),
                new ParallelAction(auto1_seg2.build())
        ));

    }
}