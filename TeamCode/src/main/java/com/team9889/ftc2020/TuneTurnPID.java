package com.team9889.ftc2020;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Eric on 6/8/2021.
 */

@TeleOp
@Config
public class TuneTurnPID extends Team9889Linear {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();
    boolean first = true;

    @Override
    public void runOpMode() throws InterruptedException {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);
        waitForStart(false);

        while (opModeIsActive()) {
            Vector2d input = new Vector2d(driverStation.getX(), driverStation.getY()).rotated(-Robot.rr.getPoseEstimate().getHeading());
            Robot.getMecanumDrive().xSpeed += input.getX();
            Robot.getMecanumDrive().ySpeed += input.getY();
            Robot.getMecanumDrive().turnSpeed -= driverStation.getSteer();

            if (timer.milliseconds() > 2000) {
                first = !first;
                timer.reset();
            }

            if (first)
                Robot.getMecanumDrive().turn(new Vector2d(10, -90));
            else {
                Robot.getMecanumDrive().turn(new Vector2d(10, 90));
            }

            while (loopTimer.milliseconds() < 15) {

            }
            loopTimer.reset();

            Robot.update();
            Robot.rr.getLocalizer().update();
        }
    }
}
