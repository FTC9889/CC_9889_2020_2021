package com.team9889.ftc2020.auto.actions.flywheel;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 6/7/2021.
 */
public class PowerShots extends Action {
    int power = 1290;

    boolean ps1, ps2, ps3, red;

    Vector2d ps1Pos = new Vector2d(73, -23), ps2Pos = new Vector2d(73, -8), ps3Pos = new Vector2d(73, -3);

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
        Robot.getInstance().flyWheel.motor.setVelocity(power);

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
            Robot.getInstance().getMecanumDrive().turn(ps1Pos);

            boolean shot = false;

            if (ready > 20)
                 shot = Robot.getInstance().getFlyWheel().shootRing();

            if (shot) {
                ps1 = false;
                ready = 0;
            }
        } else if (ps2) {
//            double angle = Robot.getInstance().getMecanumDrive().robotAngleToTarget(ps2Pos, Robot.getInstance().rr.getPoseEstimate());
            Robot.getInstance().getMecanumDrive().turn(ps2Pos);

            boolean shot = false;
            if (ready > 20)
                shot = Robot.getInstance().getFlyWheel().shootRing();

            if (shot) {
                ps2 = false;
                ready = 0;
            }
        } else if (ps3) {
//            double angle = Robot.getInstance().getMecanumDrive().robotAngleToTarget(ps3Pos, Robot.getInstance().rr.getPoseEstimate());
            Robot.getInstance().getMecanumDrive().turn(ps3Pos);

            boolean shot = false;
            if (ready > 20)
                shot = Robot.getInstance().getFlyWheel().shootRing();

            if (shot) {
                ps3 = false;
                ready = 0;
            }
        }

        if (Robot.getInstance().getMecanumDrive().headingController.getLastError() < Math.toRadians(2.75))
            ready++;
        else
            ready = 0;
    }

    @Override
    public boolean isFinished() {
        return !ps1 && !ps2 && !ps3;
    }

    @Override
    public void done() {
        Robot.getInstance().flyWheel.motor.setVelocity(0);
        Robot.getInstance().fwFlap.setPosition(.4);
        Robot.getInstance().fwLock.setPosition(1);
        Robot.getInstance().getIntake().SetPassThroughPower(0);
    }
}
