package org.firstinspires.ftc.teamcode.Teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.Mode;

import static org.firstinspires.ftc.teamcode.drive.PoseStorage.currentPose;


@TeleOp(name = "SquaredTeleop", group = "TestBot")
public class SquaredTeleop extends LinearOpMode {

    DcMotorEx shooter1 = null;
    DcMotor ringIntake = null;
    Servo extend1 = null;
    Servo pusher = null;
    Servo wobbler = null;
Servo wobbler2 = null;
    DcMotor wobbler_motor = null;
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        wobbler_motor = hardwareMap.dcMotor.get("wobble_motor");
        ringIntake = hardwareMap.dcMotor.get("intake");

        pusher = hardwareMap.servo.get("mover");
        wobbler = hardwareMap.servo.get("wobble");
	wobbler2 = hardwareMap.servo.get("wobble2");
        extend1 = hardwareMap.servo.get("extend");

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobbler_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean currentStateOfT1 = false;
        boolean currentStateOfT2 = false;
        boolean currentY2 = false;
        boolean currentA1 = false;
        boolean currentY = false;
        boolean currentDpad = false;
        boolean isUp = true;
        boolean currentX = false;

        extend1.setPosition(0);

        Gamepad g1 = gamepad1;
        Gamepad g2 = gamepad2;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(currentPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d newPose = new Pose2d(0, 0, Math.toRadians(180));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            currentStateOfT1 = g2.left_bumper;
            currentStateOfT2 = g2.right_bumper;
            currentY2 = g2.y;
            currentA1 = g1.x;
            currentY = g1.y;
            currentDpad = g1.dpad_down;
            currentX = g2.x;

            Pose2d myPose = drive.getPoseEstimate();

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if (g1.left_trigger == 1) {
                y = Math.pow(y, 3);

                x = Math.pow(x, 3);

                turn = Math.pow(turn, 3);

            }
           
            drive.setWeightedDrivePower(new Pose2d(y, x, turn));

            wobbler_motor.setPower(0.7 * g2.right_stick_y);

            if (g1.left_bumper) ringIntake.setPower(-g1.right_trigger + 1);
            else ringIntake.setPower(-g1.right_trigger);

            if (g2.right_trigger == 1) shooter1.setVelocity(-0.98 * g2.left_stick_y * 2450);
            else if (g2.left_trigger == 1) shooter1.setVelocity(-0.95 * g2.left_stick_y * 2450);
            else shooter1.setVelocity(-g2.left_stick_y * 2450);


            if (g2.left_bumper && currentStateOfT1 != false) wobbler.setPosition(0);
            if (g2.right_bumper && currentStateOfT2 != false) wobbler.setPosition(1);

            if (g2.x && currentX != false) {
                if (isUp) {
                    extend1.setPosition(0.9);
                    isUp = false;
                } else {
                    extend1.setPosition(0);
                    isUp = true;
                }
            }




            if (g1.a) pusher.setPosition(0.3);
            else pusher.setPosition(0.55);


            if (g1.y && currentY != false) {
                Trajectory toShoot = drive.trajectoryBuilder(myPose)
                        .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(180)))
                        .build();

                drive.followTrajectory(toShoot);

            }


            if (g1.x && currentA1 != false) {
                shooter1.setVelocity(0.95 * 2450);
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
                Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(0, 17, Math.toRadians(180)))
                        .build();
                Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .lineToLinearHeading(new Pose2d(0, 26, Math.toRadians(180)))
                        .build();
                Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                        .lineToLinearHeading(new Pose2d(0, 34, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(traj1);
                pusher.setPosition(0.3);
                sleep(300);
                pusher.setPosition(0.55);
                drive.followTrajectory(traj2);
                pusher.setPosition(0.3);
                sleep(300);
                pusher.setPosition(0.55);
                drive.followTrajectory(traj3);
                pusher.setPosition(0.3);
                sleep(300);
                pusher.setPosition(0.55);
            }

            Outer:
            if (g2.y && currentY2 != false) {

                if (g2.right_trigger == 1) shooter1.setVelocity(-0.98 * g2.left_stick_y * 2450);
                else if (g2.left_trigger == 1) shooter1.setVelocity(-0.95 * g2.left_stick_y * 2450);
                else shooter1.setVelocity(-g2.left_stick_y * 2450);

                if (g2.b) break Outer;
                pusher.setPosition(0.3);
                if (g2.b) break Outer;
                sleep(300);
                if (g2.b) break Outer;
                pusher.setPosition(0.55);
                if (g2.b) break Outer;
                sleep(300);

                if (g2.right_trigger == 1) shooter1.setVelocity(-0.98 * g2.left_stick_y * 2450);
                else if (g2.left_trigger == 1) shooter1.setVelocity(-0.95 * g2.left_stick_y * 2450);
                else shooter1.setVelocity(-g2.left_stick_y * 2450);

                if (g2.b) break Outer;
                pusher.setPosition(0.3);
                if (g2.b) break Outer;
                sleep(300);
                if (g2.b) break Outer;
                pusher.setPosition(0.55);
                if (g2.b) break Outer;
                sleep(300);

                if (g2.right_trigger == 1) shooter1.setVelocity(-0.98 * g2.left_stick_y * 2450);
                else if (g2.left_trigger == 1) shooter1.setVelocity(-0.95 * g2.left_stick_y * 2450);
                else shooter1.setVelocity(-g2.left_stick_y * 2450);

                if (g2.b) break Outer;
                pusher.setPosition(0.3);
                if (g2.b) break Outer;
                sleep(300);
                if (g2.b) break Outer;
                pusher.setPosition(0.55);
            }

            drive.update();

            telemetry.addData("X", myPose.getX());
            telemetry.addData("Y", myPose.getY());
            telemetry.addData("Heading", myPose.getHeading());
            telemetry.addData("Shooter Velocity", shooter1.getVelocity());
            telemetry.update();
        }


    }

}

