package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name="FourCopy1")
public class FourCopy extends LinearOpMode {
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
                .splineTo(new Vector2d(55, -19), Math.toRadians(180))
                .addTemporalMarker(0.1, () -> {

                    wobbler_motor.setPower(-0.5);
                })
                .addTemporalMarker(2.4, () -> {
                    wobbler_motor.setPower(0);


                })

                .build();

//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .splineTo(new Vector2d(54, -19), Math.toRadians(5))
//
//                .build();

//


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        sleep(200);

        drive.followTrajectory(traj2);
        shooter.setPower(0.93);
        sleep(2000);
        pusher.setPosition(0.3);
        sleep(500);
        pusher.setPosition(0.55);
        sleep(1000);
        shooter.setPower(0.93);
        pusher.setPosition(0.3);
        sleep(500);
        pusher.setPosition(0.55);
        sleep(1000);
        shooter.setPower(0.93);
        pusher.setPosition(0.3);
        sleep(500);
        pusher.setPosition(0.55);
        sleep(200);


        shooter.setPower(0);
       // drive.followTrajectory(traj3);
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