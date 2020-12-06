package com.team9889.ftc2020.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by joshua9889 on 3/28/2018.
 */

public abstract class Subsystem {
    public abstract void init(boolean auto);

    public abstract void outputToTelemetry(Telemetry telemetry);

    public abstract void update();

    public abstract void stop();
}