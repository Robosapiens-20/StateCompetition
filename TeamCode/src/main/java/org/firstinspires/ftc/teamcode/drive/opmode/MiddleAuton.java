package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@Autonomous(name="Middle Auton14")
public class MiddleAuton extends LinearOpMode {
    DcMotor wobbler_motor = null;
    Servo wobble_servo = null;
    DcMotor shooter = null;
    Servo pusher =null;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        wobbler_motor = hardwareMap.dcMotor.get("wobble_motor");
        wobble_servo = hardwareMap.servo.get("wobble");
//        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter = hardwareMap.dcMotor.get("shooter");
        pusher = hardwareMap.servo.get("mover");
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        Pose2d startPose = new Pose2d();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())

                .splineTo(new Vector2d(80,-10),Math.toRadians(180))
                .addTemporalMarker(1,()->{
                    wobble_servo.setPosition(1);
                    wobbler_motor.setPower(0.5);
                })
                .addTemporalMarker(2.3,()->{
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95,0.5,()->{
                    wobble_servo.setPosition(0.3);

                    pusher.setPosition(0.55);

                })

                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                .forward(20)
                .addTemporalMarker(0.1,()->{

                    wobbler_motor.setPower(-0.5);
                })
                .addTemporalMarker(2.4,()->{
                    wobbler_motor.setPower(0);


                })

                .build();
        Trajectory strafer = drive.trajectoryBuilder(traj2.end())

                .strafeLeft(3)


                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(traj2.end().getX(),traj2.end().getY(),Math.toRadians(0)))

                .strafeRight(20)
                .build();
//
        Trajectory traj4 =drive.trajectoryBuilder(traj3.end())
                .back(30)
                .addTemporalMarker(0.1,()->{
                    wobbler_motor.setPower(0.8);
                })
                .addTemporalMarker(1,()->{
                    wobbler_motor.setPower(0);
                })
                .build();
//
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToSplineHeading(new Pose2d(84,0,Math.toRadians(135)),Math.toRadians(0))
                .addTemporalMarker(0.1,()->{
                    wobbler_motor.setPower(-0.5);
                })
                .addTemporalMarker(0.3,()->{
                    wobbler_motor.setPower(0);
                })

                .build();
//
//


        waitForStart();

        if(isStopRequested()) return;



        drive.followTrajectory(traj1);
        sleep(200);

        drive.followTrajectory(traj2);
        delay(100);
        drive.followTrajectory(strafer);
        shooter.setPower(0.93);
        sleep(2000);
        pusher.setPosition(0.3);
        sleep(400);
        pusher.setPosition(0.55);
        sleep(800);
        shooter.setPower(0.93);
        pusher.setPosition(0.3);
        sleep(400);
        pusher.setPosition(0.55);
        sleep(800);
        shooter.setPower(0.93);
        pusher.setPosition(0.3);
        sleep(400);
        pusher.setPosition(0.55);
        sleep(100);

        shooter.setPower(0);
        drive.turn(Math.toRadians(180));
        sleep(100);

          drive.followTrajectory(traj3);
          sleep(100);
          drive.followTrajectory(traj4);
        sleep(100);
        wobble_servo.setPosition(1);
//
  sleep(500);
        drive.followTrajectory(traj5);
        sleep(100);
        wobble_servo.setPosition(0.3);
        sleep(200);

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
}