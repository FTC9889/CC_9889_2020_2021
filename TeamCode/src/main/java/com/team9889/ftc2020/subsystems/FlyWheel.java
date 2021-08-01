package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.control.controllers.FFFBMath;
import com.team9889.lib.control.controllers.PID;
import com.team9889.lib.control.controllers.PIDF;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by joshua9889 on 3/28/2018.
 */

@Config
public class FlyWheel extends Subsystem{
    public enum Mode {
        OFF, POWERSHOT1, POWERSHOT2, POWERSHOT3, POWERSHOTAUTO1, POWERSHOTAUTO2, POWERSHOTAUTO3, DEFAULT, MIDDLE, AUTO
    }
    public Mode currentMode = Mode.OFF;
    public Mode wantedMode = Mode.OFF;

    public boolean shooting = false;

    public double time = 20, rpm = 2850;
    ElapsedTime shootTimer = new ElapsedTime();
    boolean extend = false;

//    public static double P = 0.0015, I = 0, D = 0.06, F = 0, V = 0.00011, A, S = 0.25;
//    public static double P = 0.0022, I = 0, D = 0, F = 0, V = 0.0002, A, S = 0;
public static double P = 0.0007, I = 0, D = 0.2, F = 0, V = 0.0002, A, S = 0;
    public PIDF pid = new PIDF(150, 0, 20, 0.3);

    public boolean done = false;

    public boolean autoPower = false;

    public enum RampPositions {
        UP, DOWN, PS, NULL
    }
    public RampPositions currentRampPos = RampPositions.NULL;
    public RampPositions wantedRampPos = RampPositions.NULL;

    public int currentSetSpeed = 0, ready = 0;

    @Override
    public void init(boolean auto) {
//        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);

        wantedMode = Mode.OFF;

        if (auto) {
            setRampUp();
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Fly Wheel Speed", getRPM());

        telemetry.addData("Shooting", shooting);

        telemetry.addData("Ready", ready);

        telemetry.addData("Error", error);
    }

    @Override
    public void update() {
        if (!shooting) {
            rpm = distanceBasedPower();
        }

        if (currentMode == Mode.DEFAULT) {
            currentSetSpeed = (int) rpm;
        }

        if (currentRampPos != wantedRampPos) {
            switch (wantedRampPos) {
                case UP:
                    Robot.getInstance().fwFlap.setPosition(0.7);
                    break;

                case DOWN:
                    Robot.getInstance().fwFlap.setPosition(.4);
                    break;

                case PS:
                    Robot.getInstance().fwFlap.setPosition(.52);
                    break;

                case NULL:
                    break;
            }

            currentRampPos = wantedRampPos;
        }

//        if (currentMode != wantedMode) {
            switch (wantedMode) {
                case OFF:
                    setRPM(0);
                    currentSetSpeed = 0;
                    break;
                case DEFAULT:
                    setRPM(rpm);
                    currentSetSpeed = (int) rpm;
                    break;
                case MIDDLE:
                    setRPM(rpm - 300);
                    currentSetSpeed = (int) rpm - 300;
                case POWERSHOT1:
                    setRPM(2420);
                    currentSetSpeed = 2420;
                    break;
                case POWERSHOT2:
                    setRPM(2480);
                    currentSetSpeed = 2450;
                    break;
                case POWERSHOT3:
                    setRPM(1250);
                    currentSetSpeed = 1250;
                    break;
                case POWERSHOTAUTO1:
                    setRPM(1250 + 100);
                    currentSetSpeed = 1250 + 100;
                    break;
                case POWERSHOTAUTO2:
                    setRPM(1250 + 65);
                    currentSetSpeed = 1250 + 65;
                    break;
                case POWERSHOTAUTO3:
                    setRPM(1250 + 80);
                    currentSetSpeed = 1250 + 80;
                    break;
                case AUTO:
                    setRPM(2850);
                    currentSetSpeed = 2850;
                    break;
            }

            currentMode = wantedMode;
//        }

//        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);

        PMath.p = P;
        PMath.i = I;
        PMath.d = D;
        FFMath.FFConstants(V, A, S);
    }

    @Override
    public void stop() {
        Robot.getInstance().flyWheel.setPower(0);
    }

    public boolean shootRing() {
        if (shootTimer.milliseconds() > time) {
            shootTimer.reset();
            if (extend) {
                Robot.getInstance().fwArm.setPosition(0.47);
                extend = false;
                Robot.getInstance().getIntake().ringsIntaken--;
                return true;
            } else {
                Robot.getInstance().fwArm.setPosition(.62);
                extend = true;
            }
        }
        return false;
    }

    public boolean shootRing(int tolerance) {
        double error = getRPM() - currentSetSpeed;
        if (Math.abs(error) < tolerance) {
            ready++;
        } else {
            ready = 0;
        }

        if (shootTimer.milliseconds() > time) {
            shootTimer.reset();
            if (extend) {
                Robot.getInstance().fwArm.setPosition(0.47);
                extend = false;
                Robot.getInstance().getIntake().ringsIntaken--;
                return true;
            } else if (ready > 5) {
                Robot.getInstance().fwArm.setPosition(.62);
                extend = true;
                ready = 0;
            }
        }
        return false;
    }

    double error = 0;
    public boolean shootRing(int tolerance, int highTolerance) {
        error = getRPM() - currentSetSpeed;

        if (error < tolerance && error > -highTolerance) {
            ready++;
        } else {
            ready = 0;
        }

//        ready = 10;

        if (shootTimer.milliseconds() > time) {
            shootTimer.reset();
            if (extend) {
                Robot.getInstance().fwArm.setPosition(0.47);
                extend = false;
                Robot.getInstance().getIntake().ringsIntaken--;
                return true;
            } else if (ready >= 3) {
                Robot.getInstance().fwArm.setPosition(.62);
                extend = true;
                ready = 0;
            }
        }
        return false;
    }

    FFFBMath FFMath = new FFFBMath(0, 0, 0);
    PID PMath = new PID(0, 0, 0);

    public int counter = 0;
    public double power;
    public void setRPM(double rpm) {
//        Robot.getInstance().flyWheel.motor.setVelocity(rpm);
        double power = FFMath.calculateFFFBGain(rpm) + PMath.update(Robot.getInstance().flyWheel.getVelocity() / 28 * 60, rpm);
        power = CruiseLib.limitValue(power, 1, 0);
        Log.i("Power", "" + power);

        Robot.getInstance().flyWheel.setPower(power);
    }

    public void setRPM(double rpm, double tolerance, double highTolerance) {
        if (error < tolerance && error > -highTolerance) {
            power = 0;
        } else {
            double power = FFMath.calculateFFFBGain(rpm) + PMath.update(Robot.getInstance().flyWheel.getVelocity() / 28 * 60, rpm);
            power = CruiseLib.limitValue(power, 1, 0);
            Log.i("Power", "" + power);
        }

        Robot.getInstance().flyWheel.setPower(power);
    }

    public double getRPM() {
        return Robot.getInstance().flyWheel.getVelocity() / 28 * 60;
    }

    public void extend() {
        Robot.getInstance().fwArm.setPosition(0.62);
    }

    public void retract() {
        Robot.getInstance().fwArm.setPosition(0.47);
    }

    public void locked() {
        Robot.getInstance().fwLock.setPosition(1);
    }

    public void unlocked() {
        Robot.getInstance().fwLock.setPosition(.5);
    };

    public void setRampUp () {
        wantedRampPos = RampPositions.UP;
    }

    public void setRampDown () {
        wantedRampPos = RampPositions.DOWN;
    }

    public void setRampPS () {
        wantedRampPos = RampPositions.PS;
    }

    public double distanceBasedPower () {
        /*
        double initHeight = 1.16, gravity = -32.17, length = 15, y = 0;
        double vX, vY, t;
        double ringVel = 0, angle = 29;

        vX = ringVel * Math.cos(angle);
        vY = ringVel * Math.sin(angle);

        t = (Robot.getInstance().getCamera().dist / 12) / vX;

//        vY * t
        double vel = y - initHeight - (0.5 * gravity * (Math.pow(t, 2)));

        vel = vel / t;
        vel = vel / Math.sin(angle);
        return vel;

//        14.925
//        y - initHeight - (0.5 * gravity * (Math.pow(t, 2))) = (vY * t);
         */

        Pose2d pos = Robot.getInstance().rr.getLocalizer().getPoseEstimate();
        int y = Robot.getInstance().blue ? 36 : -36;
        double dist = Math.sqrt(Math.pow(63 - pos.getX(), 2) + Math.pow(y - pos.getY(), 2));
        Log.i("Dist", "" + dist);

        double rpm = (0.0579 * Math.pow(dist, 2)) - (7.8302 * dist) + 3131;

//        rpm = dist;

        rpm = CruiseLib.limitValue(rpm + 10, 3100, 2500);

        return rpm;
    }
}
