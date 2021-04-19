package com.team9889.lib.control.controllers;

import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by joshua9889 on 4/15/2018.
 */

public class PID extends FeedBackController {

    public PID(double kP, double kI, double kD){
        this.p = kP;
        this.i = kI;
        this.d = kD;
    }

    public PID(double kP, double kI, double kD, double maxIntegral){
        this.maxIntegral = maxIntegral;
        this.p = kP;
        this.i = kI;
        this.d = kD;
    }

    public double p, i, d;

    public double error_prior;
    private double integral = 0;
    private double lastTime = 0;
    double output;
    public double maxIntegral = 0;

    public boolean first = true;

    private double error = 1000000;

    double derivative;

    public static void main(String[] args) {
        PID pid = new PID(.001, 0, 0.0005);

        System.out.println(pid.update(0, 5000));
    }

    @Override
    public double update(double current, double wanted) {
        error = wanted - current;
        Log.i("Error", error + "");

        if(first){
            // P control first time
            output = p * error;
            first = false;
            try {
                RobotLog.a(String.valueOf(output));
            } catch (Exception ignored){}
        } else {
            double currentTime = System.currentTimeMillis() - lastTime;
            Log.i("Time", currentTime + "");

            integral = integral + (error *currentTime);
            if (integral > maxIntegral || i == 0)
                integral = 0;
            if (currentTime != 0)
                derivative = (error - error_prior)/currentTime;
            Log.i("Derivative", derivative + "");

//            .3104 + (-0.0814 * )
            output = (p * error) + (i * integral) + (d * derivative);
            Log.i("Output Breakdown", p + " * " + error + " * " + ", " + i + " * " + integral + ", " + d + " * " + derivative);
            Log.i("Output", output + "");
        }

        lastTime = System.currentTimeMillis();
        error_prior = error;
        return output;
    }

    public double getError(){
        return error;
    }

    public double getOutput(){return output;}
}
