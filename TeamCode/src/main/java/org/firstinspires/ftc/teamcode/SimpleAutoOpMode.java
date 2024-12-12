package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.AutoConstants;

public class SimpleAutoOpMode extends LinearOpMode {
    // Local instance of Mecanum drive class used to interface with the drive motors
    // and store chassis state and configuration
    MecanumDrive _mDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Drive
        // Starting pose defined in AutoConstants
        Pose2d initPose = new Pose2d(AutoConstants.START_POSE_X,
                                     AutoConstants.START_POSE_Y,
                                     AutoConstants.START_POSE_THETA);

        _mDrive = new MecanumDrive(hardwareMap, initPose);



        waitForStart();




        while(opModeIsActive()) {




            if(isStopRequested()) return;
        }

    }



}
