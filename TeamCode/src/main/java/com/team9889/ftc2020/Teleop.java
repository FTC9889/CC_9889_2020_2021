package com.team9889.ftc2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.teleop.AimAndShoot;
import com.team9889.ftc2020.auto.actions.teleop.PowerShots;
import com.team9889.ftc2020.subsystems.Camera;
import com.team9889.ftc2020.subsystems.Intake;
import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
@Config
public class Teleop extends Team9889Linear {
    public static double x = 103, y = 42, ay = 19.58, ax = 2.603092035700193, multiplier = 3;
    public static int timeToWait = 120;

    double setRpm = 0;

    private ElapsedTime loopTimer = new ElapsedTime(), readyTimer = new ElapsedTime();
    ElapsedTime armTimer = new ElapsedTime(), wgTimer = new ElapsedTime(), wgAutoTimer = new ElapsedTime();
    boolean flywheelOn = false;

    double ready = 0;
    boolean extend = false;

    boolean driveToPos = false, autoDrive = false, shooting = false;
    boolean driveFirst = true, turnFirst = true, psFirst = true;
    Action drive, turn, ps;

    boolean lastWGState = false;

    ElapsedTime camTimer = new ElapsedTime();

    boolean resetPressed = false;

    private PID orientationPID = new PID(1, 0, 100);


    public static double rpm = 1300, rpm12 = 1295, rpm11 = 1310, rpm10 = 1310;

    boolean wgInPos = false;
    boolean wgFirst = true;
    boolean autoAimreleased = false;
    int readyCount = 0;

    double dist = 0;

    @Override
    public void runOpMode() {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);
        waitForStart(false);

        wgTimer.reset();
        Robot.flyWheel.resetEncoder();

        while (opModeIsActive()) {
            if (!shooting) {
                if (Robot.minCurrentVoltage > 12.5) {
                    rpm = rpm12;
                } else if (Robot.minCurrentVoltage <= 12.5 && Robot.minCurrentVoltage > 11.5) {
                    rpm = rpm11;
                } else if (Robot.minCurrentVoltage <= 11.5) {
                    rpm = rpm10;
                }
            }

//            Log.v("RPM", "" + wantedRpm / ((Robot.result / multiplier) / 1.3));
//            rpm = wantedRpm / ((Robot.result / multiplier) / 1.3);


//            if (Robot.getCamera().getPosOfTarget().x != 1e10) {
//                dist = 37.852 * Math.exp(0.0192 * Robot.getCamera().scanForGoal.getPointInPixels().y);
//            }

            if (gamepad2.left_bumper && !shooting) {

                if (Robot.getCamera().currentCamState != Camera.CameraStates.GOAL) {
                    Robot.getCamera().camYPose = .7;
                    Robot.getCamera().setGoalCamPos();
                    camTimer.reset();
                }

                Robot.getCamera().setGoalCamPos();


                Robot.getCamera().setScanForGoal();

                if (camTimer.milliseconds() > 700) {

                    if (driveFirst) {
                        drive = new AimAndShoot();
                        drive.start();

                        driveFirst = false;
                    }
                    drive.update();
                }

                autoAimreleased = true;
            } else if (autoAimreleased) {
                Robot.wgGrabber.setPosition(.25);
                autoAimreleased = false;
            }

            // If not resetting imu, normal operation
            if(!driverStation.resetIMU()) {
                // Drive
                Robot.getMecanumDrive().xSpeed += driverStation.getX();
                Robot.getMecanumDrive().ySpeed += driverStation.getY();
                Robot.getMecanumDrive().turnSpeed += driverStation.getSteer();

                if (gamepad1.dpad_right) {
                    if (psFirst) {
                        ps = new PowerShots(telemetry);
                        ps.start();

                        psFirst = false;
                    }
                    if (!ps.isFinished()) {
                        ps.update();
                    } else {
                        ps.done();
                    }
                } else if (!psFirst){
                    ps.done();
                    psFirst = true;
//                    Robot.flyWheel.setPower(rpm);
                }

                Robot.getIntake().setIntakeState(driverStation.getWantedIntakeState());

//                if (driverStation.getFW() && (!Robot.getFlyWheel().psPower)){
//                    flywheelOn = true;
//                    Robot.getFlyWheel().psPower = false;
//                } else if (!driverStation.getFW() && (!Robot.getFlyWheel().psPower || gamepad1.x)) {
//                    flywheelOn = false;
//                    Robot.getFlyWheel().psPower = false;
//                }

//                if (Robot.getFlyWheel().psPower) {
//                    timeToWait = 200;
//                }

                if (Math.abs(Robot.getCamera().getPosOfTarget().x) < 0.1) {
                    ready++;
                } else if (driverStation.shoot()) {
                    ready = 0;
                }

                Robot.getFlyWheel().fireControl(driverStation.shoot() && ready > 5);

//                if ((gamepad1.right_bumper || (gamepad2.left_bumper &&
//                    ready > 5)) && flywheelOn) {
//                    Robot.getFlyWheel().setLockState(FlyWheel.RingStop.Open);
//                    if (armTimer.milliseconds() > timeToWait) {
//                        if (extend) {
//                            Robot.fwArm.setPosition(0.5);
//                            extend = false;
//                        } else {
//                            Robot.fwArm.setPosition(.65);
//                            setRpm = rpm;
//                            Robot.getIntake().resetRingCount();
//                            extend = true;
//                        }
//
//                        armTimer.reset();
//                        shooting = true;
//                    }
//                } else if (armTimer.milliseconds() > timeToWait && !Robot.getFlyWheel().psPower) {
//                    Robot.getFlyWheel().setLockState(FlyWheel.RingStop.Closed);
//                    Robot.fwArm.setPosition(0.5);
//                    extend = true;
//                    shooting = false;
//                }

                // Wobble Goal
                if (lastWGState != driverStation.getWG() || gamepad1.left_trigger > .5) {
                    wgTimer.reset();
                }

                if (!wgInPos) {
                    wgAutoTimer.reset();
                } else if (gamepad1.left_bumper){
                    wgInPos = false;
                    driverStation.codePress = true;
                }

                if (autoDrive && !wgInPos) {
                    Robot.wgLeft.setPosition(.2);
                    Robot.wgRight.setPosition(.2);
                } else if (wgInPos) {
                    if (wgAutoTimer.milliseconds() < 500) {
                        Robot.getWobbleGoal().setWGDown();
                    } else if (wgAutoTimer.milliseconds() > 500 && wgAutoTimer.milliseconds() < 1000) {
                        Robot.wgGrabber.setPosition(0.25);
                        Robot.wgLeft.setPosition(.4);
                        Robot.wgRight.setPosition(.4);
                    } else {
                        Robot.getWobbleGoal().setWGUp();
                    }
                } else if (!driverStation.getWG()) {
                    if (wgTimer.milliseconds() < 500) {
                        Robot.wgGrabber.setPosition(0.25);
                    } else {
                        Robot.getWobbleGoal().setWGDown();
                    }

                    lastWGState = false;
                } else if (driverStation.getWG()) {
                    if (wgTimer.milliseconds() < 500) {
                        Robot.getWobbleGoal().setWGUp();
                    } else {
                        Robot.wgGrabber.setPosition(.52);
                        Robot.getWobbleGoal().setWGUp();
                    }

                    lastWGState = true;
                }

                if (gamepad2.right_bumper){
                    Robot.wgGrabber.setPosition(.52);
                }

            } else {
                Robot.getMecanumDrive().setPower(0,0,0);
                Robot.getMecanumDrive().writeAngleToFile();
            }

            if (gamepad2.left_stick_button && gamepad2.right_stick_button && !resetPressed) {
                Robot.getMecanumDrive().setCurrentPose(new Pose2d(0, 0, 0));
                resetPressed = true;
            } else if (!gamepad2.left_stick_button && !gamepad2.right_stick_button) {
                resetPressed = false;
            }

            if (gamepad2.dpad_down) {
                Robot.getIntake().setArmState(Intake.ArmState.Extended);
            } else if (gamepad2.dpad_left) {
                Robot.getIntake().setArmState(Intake.ArmState.Retracted);
            }

//            if (flywheelOn) {
//                if (!Robot.getFlyWheel().psPower) {
//                    Robot.getFlyWheel().setRPM(rpm);
//                }
//            }
//            else if (!flywheelOn && !Robot.getFlyWheel().psPower) {
//                Robot.getFlyWheel().setMode(FlyWheel.Mode.OFF);
//            }

            telemetry.addData("Loop Time", loopTimer.milliseconds());
            telemetry.addData("Power Shot Power", Robot.getFlyWheel().psPower);
            telemetry.addData("Distance", dist + "");
            telemetry.addData("Rings", Robot.getIntake().ringsInHopper());

            telemetry.addData("Volts", Robot.minCurrentVoltage);
            telemetry.addData("Current RPM", setRpm);

            telemetry.addData("pixels", Robot.getCamera().scanForGoal.getPointInPixels());

            telemetry.addData("Fly Wheel Speed", (Robot.flyWheel.getVelocity() / 28) * 60);
            telemetry.addData("Odometry Adjusted : ", Robot.getMecanumDrive().getAdjustedPose());

            telemetry.addData("sensor", Robot.ringDetector.getDistance(DistanceUnit.INCH));

            Robot.outputToTelemetry(telemetry);
            telemetry.update();

            FtcDashboard.getInstance().startCameraStream(Robot.camera, 0);
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setFill("black")
                    .setStrokeWidth(2)
                    .fillRect(Robot.getMecanumDrive().getAdjustedPose().getX() - 72, Robot.getMecanumDrive().getAdjustedPose().getY() - 72, 17, 17.5);
            dashboard.sendTelemetryPacket(packet);

            Robot.getMecanumDrive().setFieldCentricPower(Robot.getMecanumDrive().xSpeed,
                    Robot.getMecanumDrive().ySpeed, Robot.getMecanumDrive().turnSpeed);

            Robot.update();

            // dt timer

//            while (loopTimer.milliseconds() < 20) {
//
//            }
            loopTimer.reset();
        }
    }
}
