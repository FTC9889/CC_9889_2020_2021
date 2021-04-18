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
        OFF, POWERSHOT1, POWERSHOT2, POWERSHOT3, POWERSHOTAUTO1, POWERSHOTAUTO2, DEFAULT
    }

    public static double P = 81, I = 0, D = 1, F = 0;
    public static int ps1 = 1260, ps2 = 1260, ps3 = 1260;
    public PIDF pid = new PIDF(81, 0, 1, 0);

    public boolean done = false;

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
                Robot.getInstance().flyWheel.motor.setVelocity(0);
                break;
            case DEFAULT:
                setRPM(2540);
                break;
            case POWERSHOT1:
                setRPM(ps1);
                break;
            case POWERSHOT2:
                setRPM(ps2);
                break;
            case POWERSHOT3:
                setRPM(ps3);
                break;
            case POWERSHOTAUTO1:
                setRPM(ps1 + 70);
            case POWERSHOTAUTO2:
                setRPM(ps1 + 50);
        }
    }

    public void setRPM(double rpm) {
        pid.update(Robot.getInstance().flyWheel.getVelocity(), rpm);

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
