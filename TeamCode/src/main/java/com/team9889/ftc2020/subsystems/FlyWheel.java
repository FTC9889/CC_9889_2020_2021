package com.team9889.ftc2020.subsystems;

import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by joshua9889 on 3/28/2018.
 */

public class FlyWheel extends Subsystem{

    private PID pid = new PID(.001, 0, 0);

    @Override
    public void init(boolean auto) {}

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("fly wheel speed: ", flySpeed);
        telemetry.addData("fly wheel set speed: ", Math.abs(.6 + pid.getOutput()));
        telemetry.addData("fly wheel pid: ", pid.getOutput());
        telemetry.addData("fly wheel pos; ", Robot.getInstance().flyWheel.getPosition());
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        Robot.getInstance().flyWheel.setPower(0);
    }

    double flySpeed = 0;
    double lastMotorPos = 0;
    public void setFlyWheelSpeed(double rpm, double time) {
        flySpeed = (((Robot.getInstance().flyWheel.getPosition() - lastMotorPos) / 28)) * ((1000 / time) * 60);

        pid.update(flySpeed, rpm);
        Robot.getInstance().flyWheel.setPower(Math.abs(.6 + pid.getOutput()));
        lastMotorPos = Robot.getInstance().flyWheel.getPosition();
    }
}
