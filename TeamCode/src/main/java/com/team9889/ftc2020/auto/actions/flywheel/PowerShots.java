package com.team9889.ftc2020.auto.actions.flywheel;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 6/7/2021.
 */
public class PowerShots extends Action {
//    int power = 1290;

    boolean ps1, ps2, ps3, red;

    ElapsedTime timer = new ElapsedTime();

    Vector2d ps1Pos = new Vector2d(73, -19), ps2Pos = new Vector2d(73, -1), ps3Pos = new Vector2d(73, -10);

    public PowerShots (boolean red) {
        ps1 = true;
        ps2 = true;
        ps3 = true;
        this.red = red;
    }

    public PowerShots (boolean ps1, boolean ps2, boolean ps3, boolean red) {
        this.ps1 = ps1;
        this.ps2 = ps2;
        this.ps3 = ps3;
        this.red = red;
    }

    @Override
    public void start() {
        Robot.getInstance().fwFlap.setPosition(.51);
        Robot.getInstance().fwLock.setPosition(.4);

        if (!red) {
            ps1Pos = new Vector2d(73, 23);
            ps2Pos = new Vector2d(73, 10);
            ps3Pos = new Vector2d(73, 2);
        }
    }

    int ready = 0;

    @Override
    public void update() {
        Robot.getInstance().getIntake().SetPassThroughPower(0.4);

        if (ps1) {
//            double angle = Robot.getInstance().getMecanumDrive().robotAngleToTarget(ps1Pos, Robot.getInstance().rr.getPoseEstimate());
            if (ready < 10)
                Robot.getInstance().getMecanumDrive().turn(ps1Pos);

            boolean shot = false;

            if (ready > 20)
                 shot = Robot.getInstance().getFlyWheel().shootRing(80, 80);

            if (shot) {
                ps1 = false;
                ready = 0;
            }
        } else if (ps2) {
//            double angle = Robot.getInstance().getMecanumDrive().robotAngleToTarget(ps2Pos, Robot.getInstance().rr.getPoseEstimate());
            if (ready < 10)
                Robot.getInstance().getMecanumDrive().turn(ps2Pos);

            boolean shot = false;
            if (ready > 20)
                shot = Robot.getInstance().getFlyWheel().shootRing(80, 80);

            if (shot) {
                ps2 = false;
                ready = 0;
            }
        } else if (ps3) {
//            double angle = Robot.getInstance().getMecanumDrive().robotAngleToTarget(ps3Pos, Robot.getInstance().rr.getPoseEstimate());
            if (ready < 10)
                Robot.getInstance().getMecanumDrive().turn(ps3Pos);

            boolean shot = false;
            if (ready > 20)
                shot = Robot.getInstance().getFlyWheel().shootRing(80, 80);

            if (shot) {
                ps3 = false;
                timer.reset();
                ready = 0;
            }
        }

        double angle = -Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES);
        if (angle < 0) {
            angle += 360;
        }

        if (Math.abs(Math.toDegrees(Robot.getInstance().getMecanumDrive().theta) - angle) < 1) {
            Log.i("Worked", "true: " + Robot.getInstance().getMecanumDrive().headingController.getLastError());
            ready++;
        }
        else {
            Log.i("Worked", "false");
            ready = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return !ps1 && !ps2 && !ps3 && timer.milliseconds() > 1000;
    }

    @Override
    public void done() {
        Robot.getInstance().getFlyWheel().wantedMode = FlyWheel.Mode.OFF;
        Robot.getInstance().fwFlap.setPosition(.4);
        Robot.getInstance().fwLock.setPosition(1);
        Robot.getInstance().getIntake().SetPassThroughPower(0);
    }
}
