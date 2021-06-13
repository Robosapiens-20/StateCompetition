package org.firstinspires.ftc.teamcode.PID_Tuning;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


@TeleOp(name = "PID Sample OpMode")
public class SamplePIDUse extends LinearOpMode {
    // Copy your PIDF Coefficients here
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 3, 13);
    private FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {
        // SETUP MOTOR //
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        Servo pusher = hardwareMap.servo.get("mover");
        // Reverse as appropriate
        // myMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // RUE limits max motor speed to 85% by default
        // Raise that limit to 100%
        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        // Turn on RUN_USING_ENCODER
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF Coefficients with voltage compensated feedforward value
        myMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // Insert whatever other initialization stuff you do here
        boolean isDownUp = false;
        boolean isDownDown = false;
        waitForStart();

        if (isStopRequested()) return;
        int multiplier = 1516;
        while (!isStopRequested()) {
            isDownUp = gamepad1.dpad_up;
            isDownDown = gamepad1.dpad_down;
            myMotor.setVelocity(-gamepad1.left_stick_y * multiplier);
            telemetry.addData("Velo", myMotor.getVelocity());
            telemetry.addData("Target", multiplier * -gamepad1.left_stick_y);
            telemetry.update();

            if (gamepad1.a) {
                pusher.setPosition(0.45);

            } else {
                pusher.setPosition(0.59);
            }
            if (gamepad1.dpad_down && isDownDown) multiplier -= 5;
            if (gamepad1.dpad_up && isDownUp) multiplier += 5;
        }
    }
}
