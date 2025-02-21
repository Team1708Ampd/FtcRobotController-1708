package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

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
//        lateralLeftElevator.setPower(power);
        lateralRightElevator.setPower(power);
    }


    // elevator actions
    public class RunElevator implements Action {

        // power at which to run
        private double mPower;

        public RunElevator(double power)
        {
            mPower = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            setPower(mPower);
            return false;
        }
    }

    // Build the action
    public Action RunElevator(double power)
    {
        return new RunElevator(power);
    }

    public class RunElevator_Timed implements Action {

        boolean initialized = false;
        // power at which to run
        private double mPower;

        // time to run (seconds)
        double mTime = 0;
        ElapsedTime t;


        public RunElevator_Timed(double power, double time)
        {
            mPower = power;
            mTime = time;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized)
            {
                initialized = true;
                setPower(mPower);
                t = new ElapsedTime();
            }

            // Return when the time is reached
            return (t.time() < mTime);
        }
    }

    // Build the action
    public Action RunElevator_Timed(double power, double time)
    {
        return new RunElevator_Timed(power, time);
    }

}
