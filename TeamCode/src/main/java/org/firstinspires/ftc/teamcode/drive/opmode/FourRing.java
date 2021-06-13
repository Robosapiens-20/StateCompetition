package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.Vector;

@Disabled
@Autonomous(name="FourRing119")
public class FourRing extends LinearOpMode {
    DcMotor wobbler_motor = null;
    Servo wobble_servo = null;
    DcMotorEx shooter = null;
    Servo pusher =null;
    DcMotor intake = null;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        wobbler_motor = hardwareMap.dcMotor.get("wobble_motor");
        wobble_servo = hardwareMap.servo.get("wobble");
//        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pusher = hardwareMap.servo.get("mover");
        intake = hardwareMap.dcMotor.get("intake");

//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Pose2d startPose = new Pose2d();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())

                .splineTo(new Vector2d(95, 13), Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    wobble_servo.setPosition(1);
                    wobbler_motor.setPower(0.5);

                })
                .addTemporalMarker(2.3, () -> {
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0.3);

                    pusher.setPosition(0.55);

                })

                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(55, -10), Math.toRadians(180))
                .addTemporalMarker(0.1, () -> {
                    shooter.setVelocity(2450);
                    wobbler_motor.setPower(-0.5);
                })
                .addTemporalMarker(2.4, () -> {
                    wobbler_motor.setPower(0);


                })

                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(80, -19), Math.toRadians(180))
                .addTemporalMarker(0.1, () -> {
                    shooter.setPower(01);

                })
                .addTemporalMarker(0.5,()->{
                    intake.setPower(1);
                })


                .build();
        Trajectory side = drive.trajectoryBuilder(new Pose2d(traj2.end().getX(),traj2.end().getY(),traj2.end().getHeading() - Math.toRadians(45)))
                .strafeLeft(9)
                .build();
        Trajectory forward = drive.trajectoryBuilder(side.end())
                .forward(45)
                .addTemporalMarker(0.9,0,()->{
                    intake.setPower(0.4);
                })

                .build();

        Trajectory toShoot = drive.trajectoryBuilder(forward.end())
                .splineToLinearHeading(new Pose2d(55,-7,Math.toRadians(175)),Math.toRadians(175))

                .build();

//
//Trajectory traj4 = drive.trajectoryBuilder(traj2.end())
//        .back(25)
//        .addTemporalMarker(0.1, () -> {
//            wobbler_motor.setPower(0.8);
//        })
//        .addTemporalMarker(1, () -> {
//            wobbler_motor.setPower(0);
//        })
//        .build();
//
//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
//                .splineToSplineHeading(new Pose2d(55,-10,Math.toRadians(-90)),Math.toRadians(0))
//                .addTemporalMarker(0.1,()->{
//                    wobbler_motor.setPower(-0.5);
//                })
//                .addTemporalMarker(0.3,()->{
//                    wobbler_motor.setPower(0);
//                })
//
//                .build();
//
//
//

        waitForStart();
        if (isStopRequested()) return;


        drive.followTrajectory(traj1);
        sleep(200);

        drive.followTrajectory(traj2);

        pusher.setPosition(0.3);
        sleep(400);
        pusher.setPosition(0.55);
        sleep(400);
        shooter.setVelocity(2450);
        pusher.setPosition(0.3);
        sleep(400);
        pusher.setPosition(0.55);
        sleep(1000);
        shooter.setVelocity(2380);
        pusher.setPosition(0.3);
        sleep(400);
        pusher.setPosition(0.55);
        sleep(200);


        shooter.setPower(0);
        drive.turn(Math.toRadians(-45));

        intake.setPower(-0.7);

        drive.followTrajectory(side);
        sleep(200);
         drive.followTrajectory(forward);
        shooter.setVelocity(2450);
        drive.followTrajectory(toShoot);
        intake.setPower(0);

        pusher.setPosition(0.3);
        sleep(400);
        pusher.setPosition(0.55);
        sleep(400);
        shooter.setVelocity(2450);
        pusher.setPosition(0.3);
        sleep(400);
        pusher.setPosition(0.55);
        sleep(1000);
        shooter.setVelocity(2380);
        pusher.setPosition(0.3);
        sleep(400);
        pusher.setPosition(0.55);
        sleep(200);

        intake.setPower(0);
        shooter.setPower(0);

        setDirectionForward(true,500,-1);
    }

    public void delay(double delayTime){
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < delayTime && opModeIsActive());
        return;
    }

    public void motorMover(DcMotor motor, double time, double power){
        while (opModeIsActive()) {
            double startTime = System.currentTimeMillis();

            while (System.currentTimeMillis() - startTime < time) {motor.setPower(power); }
            motor.setPower(0);
            break;

        }
    }

    public void setDirectionForward(boolean value, double time, double power) { //time should be in milliseconds
        String frontRightMotorName = "front_right";
        String backrightMotorName = "back_right";
        String frontleftMotorName = "front_left";
        String backleftMotorName = "back_left";




        DcMotor backRight = hardwareMap.dcMotor.get(backrightMotorName);
        DcMotor backLeft = hardwareMap.dcMotor.get(backleftMotorName);
        DcMotor frontRight = hardwareMap.dcMotor.get(frontRightMotorName);
        DcMotor frontLeft = hardwareMap.dcMotor.get(frontleftMotorName);

        double startTime = System.currentTimeMillis();
        if (value) {//forward
            telemetry.addData("forward", "starting");
            telemetry.update();
            backRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setPower(-power);
            backLeft.setPower(-power);
            frontLeft.setPower(-power);
            frontRight.setPower(-power);


            while (opModeIsActive()) {
                if ((System.currentTimeMillis() - startTime) >= time) {
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    telemetry.addData("stopped", "moving on");
                    telemetry.update();
                    break;
                }
            }

        } else {//backward
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setPower(-power);
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);

            while (opModeIsActive()) {
                if ((System.currentTimeMillis() - startTime) >= time) {
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    break;
                }
            }
        }
    }

}