package com.team9889.ftc2020;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
public class Teleop extends Team9889Linear {

    private ElapsedTime loopTimer = new ElapsedTime();
    ElapsedTime armTimer = new ElapsedTime();
    boolean on = false;

    double lastTime;
    boolean extend = false;

    @Override
    public void runOpMode() {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);
        waitForStart(false);

        while (opModeIsActive()) {
            // If not resetting imu, normal operation
            if(!driverStation.resetIMU()) {
                // Drive
                Robot.getMecanumDrive().setFieldCentricPower(driverStation.getX() / driverStation.getSlowDownFactor(),
                        driverStation.getY() / driverStation.getSlowDownFactor(), driverStation.getSteer() / driverStation.getSlowDownFactor());

                if (driverStation.getStartIntaking()) {
                    Robot.getIntake().Intake();
                } else if (driverStation.getStopIntaking()) {
                    Robot.getIntake().Stop();
                } else if (driverStation.getStartOuttaking()) {
                    Robot.getIntake().Outtake();
                }

                if (gamepad1.x){
                    on = true;
                } else if (gamepad1.start) {
                    on = false;
                }

                if (gamepad1.right_bumper && on) {
                    if (armTimer.milliseconds() > 200) {
                        if (extend) {
                            Robot.fwArm.setPosition(1);
                            extend = false;
                        } else {
                            Robot.fwArm.setPosition(0);
                            extend = true;
                        }

                        armTimer.reset();
                    }
                } else if (armTimer.milliseconds() > 200) {
                    Robot.fwArm.setPosition(0);
                    extend = true;
                }

                if (!driverStation.getWG()) {
                    Robot.wgLeft.setPosition(.8);
                    Robot.wgRight.setPosition(.8);
                } else if (driverStation.getWG()) {
                    Robot.wgLeft.setPosition(0.3);
                    Robot.wgRight.setPosition(0.3);
                }

                if (driverStation.getWGG()) {
                    Robot.wgGrabber.setPosition(.3);
                } else if (!driverStation.getWGG()) {
                    Robot.wgGrabber.setPosition(0.1);
                }

                if (gamepad1.dpad_right){
                    Robot.wgLeft.setPosition(.5);
                    Robot.wgRight.setPosition(.5);
                    Robot.wgGrabber.setPosition(.3);
                }

            } else {
                Robot.getMecanumDrive().setPower(0,0,0);
                Robot.getMecanumDrive().writeAngleToFile();
            }

            while (loopTimer.milliseconds() < 20) {

            }

            if (on) {
                Robot.getFlyWheel().setFlyWheelSpeed(5700, loopTimer.milliseconds());
            }
            else if (!on) {
                Robot.flyWheel.setPower(0);
            }

            telemetry.addData("Loop Time", loopTimer.milliseconds());
            // telemetry.addData("left intake", -Robot.intakeLeft.getPosition());
            // telemetry.addData("right intake", -Robot.intakeRight.getPosition());
            telemetry.addData("Fly Wheel", Robot.flyWheel.getPosition());
            Robot.outputToTelemetry(telemetry);
            telemetry.update();

            Robot.update();

            // dt timer
            loopTimer.reset();
        }
    }
}
