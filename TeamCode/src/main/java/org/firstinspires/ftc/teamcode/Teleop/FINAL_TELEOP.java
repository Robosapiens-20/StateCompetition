package org.firstinspires.ftc.teamcode.Teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Current_Auton.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.Mode;

import static org.firstinspires.ftc.teamcode.drive.PoseStorage.currentPose;


@TeleOp(name = "Final Teleop", group = "TestBot")
public class FINAL_TELEOP extends LinearOpMode {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 3, 13);
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

    SampleMecanumDrive drive;

    int multiplier;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        MotorConfigurationType motorConfigurationType = shooter1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter1.setMotorType(motorConfigurationType);
        wobbler_motor = hardwareMap.dcMotor.get("wobble_motor");
        ringIntake = hardwareMap.dcMotor.get("intake");

        pusher = hardwareMap.servo.get("mover");
        wobbler = hardwareMap.servo.get("wobble");
        wobbler2 = hardwareMap.servo.get("wobble2");
        extend1 = hardwareMap.servo.get("extend");

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));
        wobbler_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        boolean currentStateOfT1 = false;
        boolean currentStateOfT2 = false;
        boolean currentY2 = false;
        boolean currentA1 = false;
        boolean currentY = false;
        boolean currentDpad = false;
        boolean isUp = true;
        boolean currentX = false;
        boolean isDownDown = false;
        boolean isDownUp = false;

        extend1.setPosition(0.9);

        Gamepad g1 = gamepad1;
        Gamepad g2 = gamepad2;

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(currentPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d newPose = new Pose2d(0, 0, Math.toRadians(180));
        multiplier = 1410;
        double psMultiplier = 1315;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Target Velo: ", multiplier);

            //  double elapsed = System.currentTimeMillis() - startTime;
            currentStateOfT1 = g2.left_bumper;
            currentStateOfT2 = g2.right_bumper;
            currentY2 = g2.y;
            currentA1 = g1.x;
            currentY = g1.y;
            currentDpad = g1.dpad_down;
            currentX = g2.x;
            isDownDown = g2.dpad_down;
            isDownUp = g2.dpad_up;

            Pose2d myPose = drive.getPoseEstimate();

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if (g2.left_stick_y != 0) {
                turn *= 0.7;
            }

            drive.setWeightedDrivePower(new Pose2d(y, x, turn));

            wobbler_motor.setPower(0.7 * g2.right_stick_y);

            ringIntake.setPower(-g1.right_trigger + g1.left_trigger);

            if (gamepad2.dpad_down && isDownDown) {
                if (g2.left_trigger == 1) psMultiplier -= 5;
                else multiplier -= 5;
            }
            if (gamepad2.dpad_up && isDownUp) {
                if (g2.left_trigger == 1) psMultiplier += 5;
                else multiplier += 5;
            }

            if (g2.left_trigger == 1) shooter1.setVelocity(psMultiplier);
            else shooter1.setVelocity(-g2.left_stick_y * multiplier);

            if (g2.b && g2.left_trigger == 1) psMultiplier = 1315;
            else if (g2.b) multiplier = 1400;

            if (g1.dpad_down && currentDpad != false) {
                drive.setPoseEstimate(newPose);
            }

            if (g2.left_bumper && currentStateOfT1 != false) {
                wobbler.setPosition(0);
                wobbler2.setPosition(0);
            }
            if (g2.right_bumper && currentStateOfT2 != false) {
                wobbler.setPosition(1);
                wobbler2.setPosition(1);
            }

            if (g1.left_bumper || g2.a) pusher.setPosition(0.45);
            else pusher.setPosition(0.58);

            if (g2.x) extend1.setPosition(0);
            else extend1.setPosition(0.9);

            if (g1.y && currentY != false) {
                Trajectory toShoot = drive.trajectoryBuilder(myPose)
                        .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(180)))
                        .build();

                drive.followTrajectory(toShoot);

            }


            if (g1.x && currentA1 != false) {
                shooter1.setVelocity(1315);
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
                Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(0, 17, Math.toRadians(180)))
                        .build();
                Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .lineToLinearHeading(new Pose2d(0, 24.5, Math.toRadians(180)))
                        .build();
                Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                        .lineToLinearHeading(new Pose2d(0, 33, Math.toRadians(180)))
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
                shootRings(3);
            }

            drive.update();
            telemetry.addData("Powershot Target", psMultiplier);
            telemetry.addData("X coordinate", myPose.getX());
            telemetry.addData("Y coordinate", myPose.getY());
            telemetry.addData("Heading", myPose.getHeading());
            telemetry.addData("Shooter Velocity", shooter1.getVelocity());
            telemetry.update();
        }


    }

    public void shootRings(int num) {
        drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x,-gamepad1.right_stick_x));
        shooter1.setVelocity(multiplier);
        drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x,-gamepad1.right_stick_x));
        sleep(500);
        drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x,-gamepad1.right_stick_x));
        for (int i = 1; i <= num; i++) {
            sleep(100);
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x,-gamepad1.right_stick_x));
            pusher.setPosition(Constants.frontShooterServo);
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x,-gamepad1.right_stick_x));
            sleep(200);
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x,-gamepad1.right_stick_x));
            pusher.setPosition(Constants.backShooterServo);
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x,-gamepad1.right_stick_x));
        }
    }

}
