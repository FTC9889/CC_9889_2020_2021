package com.team9889.ftc2019.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.team9889.ftc2019.Constants;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.RunningAverage;
import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;
import java.util.Timer;

/**
 * Created by joshua9889 on 3/28/2018.
 */

public class FlyWheel extends Subsystem{

    PID pid = new PID(.001, 0, 0);

    @Override
    public void init(boolean auto) {}

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("fly wheel speed: ", flySpeed);
        telemetry.addData("fly wheel set speed: ", Math.abs(.6 + pid.getOutput()));
        telemetry.addData("fly wheel pid: ", pid.getOutput());
        telemetry.addData("fly wheel pos; ", Robot.getInstance().flyWheel.getPosition());
    }

    @Override
    public void update() {}

    double flySpeed = 0;
    double lastMotorPos = 0;
    public void setFlyWheelSpeed(double rpm, double time) {
        flySpeed = (((Robot.getInstance().flyWheel.getPosition() - lastMotorPos) / 28)) * ((1000 / time) * 60);

        pid.update(flySpeed, rpm);
        Robot.getInstance().flyWheel.setPower(Math.abs(.6 + pid.getOutput()));
        lastMotorPos = Robot.getInstance().flyWheel.getPosition();
    }
}
