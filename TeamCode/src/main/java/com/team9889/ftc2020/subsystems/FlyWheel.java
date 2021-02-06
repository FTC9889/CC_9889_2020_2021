package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.team9889.lib.CruiseLib;
import com.team9889.lib.android.FileReader;
import com.team9889.lib.android.FileWriter;
import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Array;
import java.util.Arrays;

/**
 * Created by joshua9889 on 3/28/2018.
 */

public class FlyWheel extends Subsystem{

//    public PID pid = new PID(.001, 0, 0.0005);
    public PID pid = new PID(.000008, 0, 0.01);
//    .000008, 0, 0

    @Override
    public void init(boolean auto) {
        FileWriter fwWriter = new FileWriter("flywheelSpeed.txt");
        fwWriter.write("0");
        fwWriter.close();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        Log.i("current fly wheel speed", "" + flySpeed);
        Log.i("fly wheel set a speed: ", "" + Math.abs(wantedFWSpeed));
        Log.i("fly wheel set speed: ", "" + wantedFWSpeed);
        Log.i("fly wheel pid: ", "" + pid.getOutput());
        Log.i("fly wheel pos; ", "" + Robot.getInstance().flyWheel.getPosition());
        Log.i("fly last pos", "" + lastMotorPos);
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
        Log.i("LT", "" + time);
        flySpeed = (((Robot.getInstance().flyWheel.getPosition() - lastMotorPos) / 28)) * ((1000 / time) * 60);

        pid.update(flySpeed, rpm);
//        Robot.getInstance().flyWheel.setPower(Math.abs(.6 + pid.getOutput()));
//        wantedFWSpeed += (pid.getOutput() / 400);
        wantedFWSpeed += (pid.getOutput());
        wantedFWSpeed = CruiseLib.limitValue(wantedFWSpeed, 1, 0);

//        if (counter < 20) {
//            Robot.getInstance().flyWheel.setPower(.53);
//        } else if (counter == 20) {
//            wantedFWSpeed = .53;
//        } else {
            Robot.getInstance().flyWheel.setPower(Math.abs(wantedFWSpeed));
//        }

        lastMotorPos = Robot.getInstance().flyWheel.getPosition();

        counter++;
    }
}
