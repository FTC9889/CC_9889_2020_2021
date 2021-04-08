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

@Config
public class FlyWheel extends Subsystem{
    public enum Mode {
        OFF, POWERSHOT1, POWERSHOT2, DEFAULT
    }

    public static double P = 200, I = 0, D = 11, F = .15;
    public PIDF pid = new PIDF(0.0008, 0, 0.04, .15);

    public boolean psPower = false;

    @Override
    public void init(boolean auto) {
        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);

        if (auto) {
            Robot.getInstance().fwLock.setPosition(1);
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {}

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

    public void setMode(Mode mode) {
        switch (mode) {
            case OFF:
                setRPM(0);
                break;
            case DEFAULT:
                setRPM(2540);
                break;
            case POWERSHOT1:
                setRPM(1130);
                break;
            case POWERSHOT2:
                setRPM(1150);
                break;
        }
    }

    public void setRPM(double rpm) {
        Robot.getInstance().flyWheel.motor.setVelocity(rpm);
    }

    public void fire() {

    }

    public void extend() {
        Robot.getInstance().fwArm.setPosition(0.45);
    }

    public void retract() {
        Robot.getInstance().fwArm.setPosition(1.0);
    }
}
