package com.team9889.ftc2020.auto.actions.flywheel;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.control.controllers.PID;

/**
 * Created by Eric on 4/5/2021.
 */

@Config
public class PowerShotsTeleOp extends Action {
    int power = 1370;

    public static PID camOrientationPID = new PID(0.6, 0, 30);

    boolean ps1, ps2, ps3;

    ElapsedTime timer = new ElapsedTime();

    Vector2d ps1Pos = new Vector2d(73, -23), ps2Pos = new Vector2d(73, -10), ps3Pos = new Vector2d(73, -2);

    public PowerShotsTeleOp() {
        ps1 = true;
        ps2 = true;
        ps3 = true;
    }

    public PowerShotsTeleOp(boolean ps1, boolean ps2, boolean ps3) {
        this.ps1 = ps1;
        this.ps2 = ps2;
        this.ps3 = ps3;
    }

    @Override
    public void start() {
        Robot.getInstance().flyWheel.motor.setVelocity(power);

        Robot.getInstance().fwFlap.setPosition(.51);
        Robot.getInstance().fwLock.setPosition(.4);

        Robot.getInstance().getCamera().setTelePS1CamPos();
    }

    int ready = 0;

    @Override
    public void update() {
        Robot.getInstance().getIntake().SetPassThroughPower(0.4);

        if (ps1) {
            Robot.getInstance().getCamera().setTelePS1CamPos();

            if (timer.milliseconds() > 500) {
                if (ready == 0) {
                    turn();
                }

                ready();

                boolean shot = false;

                if (ready > 10)
                    shot = Robot.getInstance().getFlyWheel().shootRing();

                if (shot) {
                    ps1 = false;
                    ready = 0;
                    Robot.getInstance().getCamera().setTelePS3CamPos();
                    timer.reset();
                }
            }
        } else if (ps3) {
            Robot.getInstance().getCamera().setTelePS3CamPos();

            if (timer.milliseconds() > 500) {
                if (ready == 0) {
                    turn();
                }

                ready();

                boolean shot = false;

                if (ready > 10)
                    shot = Robot.getInstance().getFlyWheel().shootRing();

                if (shot) {
                    ps3 = false;
                    ready = 0;
                    Robot.getInstance().getCamera().setTelePS4CamPos();
                    timer.reset();
                }
            }
        } else if (ps2) {
            if (timer.milliseconds() < 500) {
                Robot.getInstance().getCamera().setTelePS4CamPos();
            }
            if (ready == 0) {
                turn();
            }

            if (timer.milliseconds() > 500 && timer.milliseconds() < 1000) {
//                turn();
            } else if (timer.milliseconds() >= 1000 && timer.milliseconds() < 1500) {
                Robot.getInstance().getCamera().setTelePS2CamPos();
            } else if (timer.milliseconds() >= 1500) {
                ready();

                boolean shot = false;

                if (ready > 10)
                    shot = Robot.getInstance().getFlyWheel().shootRing();

                if (shot) {
                    ps2 = false;
                    ready = 0;
                }
            }

        }
    }

    void turn () {
        double cameraTarget = Robot.getInstance().getCamera().getPosOfTarget().x;

        double speed = 0;
        if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) >= 0.05){
            speed = -camOrientationPID.update(cameraTarget, 0);
        }

        Robot.getInstance().getMecanumDrive().turnSpeed += (speed);
    }

    void ready() {
        if (Math.abs(camOrientationPID.getError()) < 0.1)
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
