package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ClawSub {

    public Servo clawServo;
    public CRServo leftRotate;
    public CRServo rightRotate;

    public void setRotate(double position) {
        leftRotate.setPower(position);
        rightRotate.setPower(position);
    }

    public void setClawServo(double position) {
        clawServo.setPosition(position);
    }

    // Claw Actions
    public class SetClaw implements Action {

        // Final position for claw
        private double mPosition;

        public SetClaw(double pos)
        {
            mPosition = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            setRotate(mPosition);
            return false;
        }
    }

    // Build the action
    public Action SetClaw(double pos)
    {
        return new SetClaw(pos);
    }
}
