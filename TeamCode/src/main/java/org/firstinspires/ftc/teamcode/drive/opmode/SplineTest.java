package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );

//        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(30, 30), 0)
//                .build();
//
//        drive.followTrajectory(traj2);
//
//        sleep(2000);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj2.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
//
//        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(30, 30), 0)
//                .build();
//
//        drive.followTrajectory(traj3);
//
//        sleep(2000);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj3.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
//
//        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(30, 30), 0)
//                .build();
//
//        drive.followTrajectory(traj4);
//
//        sleep(2000);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj4.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
}