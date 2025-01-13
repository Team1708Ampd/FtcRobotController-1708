package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@Autonomous(name = "BLUE_AUTO_1")
public class itdAutoOpMode extends LinearOpMode {

    // SUBSYSTEM MODULES
    // Mecanum drive
    MecanumDrive _drive;
    // Elevator
    Elevator _elevator;
    // Intake
    Intake _intake;

    // Telemetry objects
    TelemetryPacket _packet;

    public void runOpMode() throws InterruptedException {
        _packet = new TelemetryPacket();

        // Set up the initial pose
        Pose2d initPose = new Pose2d(AutoConstants.START_POSE_X,
                                     AutoConstants.START_POSE_Y,
                                     AutoConstants.START_POSE_THETA);

        // Init Subsystem Modules
        _drive = new MecanumDrive(hardwareMap, initPose);

        _elevator = new Elevator();
        _intake = new Intake();

        // Init Vision

        // TRAJECTORIES
        // Create trajectories for all 3 starting positions
        // Note - A single trajectory will be defined for each starting configuration
        // and Pose Mapping will be utilized to translate these trajectories between
        // the Red and Blue Alliance sides

        // Start 1
        TrajectoryActionBuilder auto1 = _drive.actionBuilder(initPose);

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
        chosenTrajectory = auto1.build();

        // Run all actions
        Actions.runBlocking(new SequentialAction(
                chosenTrajectory
        ));

    }
}
