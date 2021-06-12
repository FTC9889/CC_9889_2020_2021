package com.team9889.ftc2020.auto.actions.teleop;

import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 8/26/2020.
 */
public class TurnToAngle extends Action {
    private double angle;
    private PID orientationPID = new PID(0.03, 0, 2.25);

    private int angleCounter = 0;

    public TurnToAngle(double angle) {
        this.angle = angle;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        double wantedAngle;
        if (angle > 180){
            wantedAngle = angle - 360;
        }else if (angle < -180){
            wantedAngle = angle + 360;
        }else
            wantedAngle = angle;

        double turn = wantedAngle - Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES);

        if (turn > 180){
            turn = turn - 360;
        }else if (turn < -180){
            turn = turn + 360;
        }
        turn *= -1;

        double rotation = orientationPID.update(turn, 0);

        double maxPower = 1;

        rotation = CruiseLib.limitValue(rotation, -.15, -maxPower, .15, maxPower);
        Robot.getInstance().getMecanumDrive().turnSpeed += rotation;
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(orientationPID.getError()) < 0.5)
            angleCounter++;
        else angleCounter = 0;

        return (angleCounter > 3);
    }

    @Override
    public void done() {
    }
}
