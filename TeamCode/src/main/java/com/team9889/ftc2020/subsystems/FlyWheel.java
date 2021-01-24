package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by joshua9889 on 3/28/2018.
 */

public class FlyWheel extends Subsystem{

    public PID pid = new PID(.001, 0, 0.0005);

    @Override
    public void init(boolean auto) {}

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        Log.i("fly wheel speed: ", "" + flySpeed);
        Log.i("fly wheel set speed: ", "" + Math.abs(.6 + pid.getOutput()));
        Log.i("fly wheel pid: ", "" + pid.getOutput());
        Log.i("fly wheel pos; ", "" + Robot.getInstance().flyWheel.getPosition());
        Log.i("wanted fly wheel; ", "" + wantedFWSpeed);
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        Robot.getInstance().flyWheel.setPower(0);
    }

    double flySpeed = 0;
    public double lastMotorPos = 0;
    public double wantedFWSpeed = 0;
    public int counter = 0;
    public void setFlyWheelSpeed(double rpm, double time) {
        flySpeed = (((Robot.getInstance().flyWheel.getPosition() - lastMotorPos) / 28)) * ((1000 / time) * 60);

        pid.update(flySpeed, rpm);
        Robot.getInstance().flyWheel.setPower(Math.abs(.6 + pid.getOutput()));
//        wantedFWSpeed += (pid.getOutput() / 400);

//        Robot.getInstance().flyWheel.setPower(Math.abs(wantedFWSpeed));
        lastMotorPos = Robot.getInstance().flyWheel.getPosition();

        counter++;
    }
}
