package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.util.VisionUtil;

/**
 * Live P/F tuning for the shooter flywheel.
 *
 * <p>D-pad up/down: P. D-pad left/right: F. X: cycle step size. Y: toggle target velocity.
 * A/B: intake on/off. Left bumper: toggle computing P from target distance (off by default, so
 * manual tuning isn't overwritten).
 */
@TeleOp(name = "Flywheel PIDF Tuner", group = "Test")
public class FlywheelPidfTuner extends OpMode {

    // Velocities are in encoder ticks per second
    private static final double HIGH_VELOCITY = 1500;
    private static final double LOW_VELOCITY = 1300;

    private static final double HOOD_POSITION = 0.4;
    private static final double[] STEP_SIZES = {10.0, 1.0, 0.1, 0.001, 0.0001};

    // The "ready" window is asymmetric around center
    private static final double AIM_MIN_TX = -2.0;
    private static final double AIM_MAX_TX = 2.5;

    private final Intake intake = new Intake();
    private DcMotorEx flywheel;
    private Limelight3A limelight;
    private IMU imu;

    private double targetVelocity = HIGH_VELOCITY;
    private double p = 0;
    private double f = 0;
    private int stepIndex = 1;
    /** When on, P is computed from target distance instead of the D-pad value. */
    private boolean useDistanceP = false;

    @Override
    public void init() {
        intake.init(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        hardwareMap.get(Servo.class, "hoodServo").setPosition(HOOD_POSITION);
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, 0, f));
        telemetry.addLine("Init Complete");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        handleInput();

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, 0, f));
        flywheel.setVelocity(targetVelocity);
        double currentVelocity = flywheel.getVelocity();

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currentVelocity);
        telemetry.addData("Error", "%.2f", targetVelocity - currentVelocity);
        telemetry.addLine("----------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", p);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", f);
        telemetry.addData("Step Size", "%.4f (X Button)", STEP_SIZES[stepIndex]);

        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            if (tx > AIM_MIN_TX && tx < AIM_MAX_TX) {
                telemetry.addLine("READY TO SHOOT");
            }
            double distance = VisionUtil.distanceFromTargetArea(result.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.addData("Botpose", result.getBotpose_MT2());

            if (useDistanceP) {
                p = 0.000610948 * distance * distance + 0.097263 * distance + 31.22434;
            }
        }
        telemetry.addData("Distance-based P", "%s (Left Bumper)", useDistanceP ? "ON" : "OFF");
    }

    private void handleInput() {
        if (gamepad1.aWasPressed()) {
            intake.start();
        }
        if (gamepad1.bWasPressed()) {
            intake.stop();
        }
        if (gamepad1.yWasPressed()) {
            targetVelocity = (targetVelocity == HIGH_VELOCITY) ? LOW_VELOCITY : HIGH_VELOCITY;
        }
        if (gamepad1.leftBumperWasPressed()) {
            useDistanceP = !useDistanceP;
        }
        if (gamepad1.xWasPressed()) {
            stepIndex = (stepIndex + 1) % STEP_SIZES.length;
        }
        if (gamepad1.dpadUpWasPressed()) {
            p += STEP_SIZES[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            p -= STEP_SIZES[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            f += STEP_SIZES[stepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            f -= STEP_SIZES[stepIndex];
        }
    }
}
