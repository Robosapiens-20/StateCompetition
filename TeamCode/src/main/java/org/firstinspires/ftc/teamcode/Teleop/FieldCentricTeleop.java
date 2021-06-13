package org.firstinspires.ftc.teamcode.Teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@TeleOp(name = "FieldCentricTeleop", group = "TestBot")
public class FieldCentricTeleop extends LinearOpMode {

    DcMotorEx shooter1 = null;
    DcMotor ringIntake = null;
    Servo extend1 = null;
    Servo pusher = null;
    Servo wobbler = null;
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
        extend1 = hardwareMap.servo.get("extend");

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobbler_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean currentStateOfT1 = false;
        boolean currentStateOfT2 = false;
        boolean currentY2 = false;

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

            Pose2d myPose = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-myPose.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            wobbler_motor.setPower(0.7 * g2.right_stick_y);

            if (g2.dpad_down && ringIntake.getPower() == 0) ringIntake.setPower(-g1.right_trigger + g1.left_trigger - 1);
            else ringIntake.setPower(-g1.right_trigger + g1.left_trigger);

            if (g2.right_trigger == 1) shooter1.setVelocity(-0.98 * g2.left_stick_y * 2450);
            else if (g2.left_trigger == 1) shooter1.setVelocity(-0.95 * g2.left_stick_y * 2450);
            else shooter1.setVelocity(-g2.left_stick_y * 2450);

            if (g2.a) pusher.setPosition(0.3);
            else pusher.setPosition(0.55);

            if (g2.left_bumper && currentStateOfT1 != false) wobbler.setPosition(0.3);
            if (g2.right_bumper && currentStateOfT2 != false) wobbler.setPosition(1);

            if (g1.left_bumper || g2.x) extend1.setPosition(0.9);
            else extend1.setPosition(0);

            Outer: if (g2.y && currentY2 != false) {

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

