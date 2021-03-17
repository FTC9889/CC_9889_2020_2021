package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.android.FileReader;
import com.team9889.lib.android.FileWriter;
import com.team9889.lib.control.controllers.PID;
import com.team9889.lib.control.controllers.PIDF;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Array;
import java.util.Arrays;

/**
 * Created by joshua9889 on 3/28/2018.
 */

public class FlyWheel extends Subsystem{

//    public PID pid = new PID(.001, 0, 0.0005);
    public static double P = 200, I = 0, D = 10, F = .15;

//    public PID pid = new PID(0.00001, 0.0000000001, 0.02);
    public PIDF pid = new PIDF(0.0008, 0, 0.04, .15);
//    1.5, 0, 10, .4
//    .000008, 0, 0

    public boolean psPower = false;

    @Override
    public void init(boolean auto) {
        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);
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
        pid.p = P;
        pid.i = I;
        pid.d = D;
        pid.kFF = F;

        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);
    }

    @Override
    public void stop() {
        Robot.getInstance().flyWheel.setPower(0);
    }

    public double flySpeed = 0;
    public double lastMotorPos = 0;
    public double wantedFWSpeed = 0;
    public int counter = 0;
    public void setFlyWheelSpeed(double rpm, double time) {
        Log.i("LT", "" + time);
//        flySpeed = (((Robot.getInstance().flyWheel.getPosition() - lastMotorPos) / 28)) * ((1000 / time) * 60);

        flySpeed = (Robot.getInstance().flyWheel.getVelocity() / 28) * 60;

        pid.update(flySpeed, rpm);
//        Robot.getInstance().flyWheel.setPower(Math.abs(.6 + pid.getOutput()));
//        wantedFWSpeed += (pid.getOutput() / 400);

        if (Math.abs(pid.getError()) > 150) {
            wantedFWSpeed += (pid.getOutput());
            wantedFWSpeed = CruiseLib.limitValue(wantedFWSpeed, 1, 0);

//            Robot.getInstance().flyWheel.setPower(Math.abs(wantedFWSpeed));
        }


        lastMotorPos = Robot.getInstance().flyWheel.getPosition();

        counter++;
    }
}
