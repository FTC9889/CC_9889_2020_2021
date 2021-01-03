package com.team9889.ftc2020.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Eric on 8/19/2019.
 */

public class Intake extends Subsystem {

    @Override
    public void init(boolean auto) {

    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        Stop();
    }

    public void SetIntakePower(double power){
        Robot.getInstance().intakeLeft.setPower(power);
        Robot.getInstance().intakeRight.setPower(power);
    }
    public void Intake(){
//        SetIntakePower(0.45);
        SetIntakePower(0.8);
    }
    public void Outtake(){
        SetIntakePower(-0.45);
    }
    public void Stop(){
        SetIntakePower(0);
    }
}