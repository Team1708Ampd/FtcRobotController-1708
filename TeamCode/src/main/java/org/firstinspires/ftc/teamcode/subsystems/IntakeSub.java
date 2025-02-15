package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSub {
    public CRServo intakeServo;
    public Servo intakeRotation;

    public void setIntake(double power) {
        intakeServo.setPower(power);
    }

    public void setRotate(double position) {
        intakeRotation.setPosition(position);
    }

    public class RunIntake implements Action {

        // power at which to run
        private double mPower;

        public RunIntake(double power)
        {
            mPower = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            setIntake(mPower);
            return false;
        }
    }

    // Build the action
    public Action RunIntake(double power)
    {
        return new RunIntake(power);
    }

    public class RunIntake_Timed implements Action {

        boolean initialized = false;
        // power at which to run
        private double mPower;

        // time to run (seconds)
        double mTime = 0;
        ElapsedTime t;


        public RunIntake_Timed(double power, double time)
        {
            mPower = power;
            mTime = time;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized)
            {
                setIntake(mPower);
                t = new ElapsedTime();
            }

            // Return when the time is reached
            return (t.time() < mTime);
        }
    }

    // Build the action
    public Action RunIntake_Timed(double power, double time)
    {
        return new RunIntake_Timed(power, time);
    }

}
