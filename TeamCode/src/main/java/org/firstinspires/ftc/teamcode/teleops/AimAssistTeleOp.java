package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.VisionUtil;

/**
 * Same controls as {@link BasicTeleOp}, plus Limelight telemetry that tells the driver which way
 * to turn to line up a shot.
 */
@TeleOp(name = "Aim Assist TeleOp", group = "Basic")
public class AimAssistTeleOp extends LinearOpMode {

    // Hood servo positions (0 is hood fully up, 1 is hood down)
    private static final double HOOD_CLOSE = 0.41; // originally 0.51 in V1 testing
    private static final double HOOD_FAR = 0.31;   // originally 0 in V1 testing

    private static final double GATE_CLOSED = 1.0;
    private static final double GATE_OPEN = 0.4;

    // Shooter velocities are in encoder ticks per second
    private static final double CLOSE_VELOCITY = 1300;
    private static final double FAR_VELOCITY = 1500;
    private static final double SHOOTER_REVERSE_POWER = -0.1;

    private static final PIDFCoefficients INITIAL_PIDF = new PIDFCoefficients(130, 0, 0, 0);
    private static final PIDFCoefficients FAR_PIDF = new PIDFCoefficients(130, 0, 0, 12.82);
    private static final PIDFCoefficients CLOSE_PIDF = new PIDFCoefficients(130, 0, 0, 13.154);

    private static final double TRIGGER_THRESHOLD = 0.9;

    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();
    private DcMotorEx shooter;
    private Servo hood;
    private Servo gate;
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        hood = hardwareMap.get(Servo.class, "hoodServo");
        gate = hardwareMap.get(Servo.class, "gateServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, INITIAL_PIDF);
        hood.setPosition(HOOD_CLOSE);
        gate.setPosition(GATE_CLOSED);

        telemetry.addLine("Init Complete");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            updateIntake();
            updateShooter();

            if (gamepad2.a || gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                feedShooter();
            }

            showAimHint();
            telemetry.update();
        }
    }

    private void updateIntake() {
        if (gamepad1.b) {
            intake.stop();
            gate.setPosition(GATE_CLOSED);
        } else if (gamepad1.a) {
            intake.start();
            gate.setPosition(GATE_CLOSED);
        } else if (gamepad1.x) {
            intake.reverse();
            gate.setPosition(GATE_CLOSED);
        }
    }

    private void updateShooter() {
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FAR_PIDF);
            shooter.setVelocity(FAR_VELOCITY);
            hood.setPosition(HOOD_FAR);
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            shooter.setPower(0);
            hood.setPosition(HOOD_CLOSE);
            gate.setPosition(GATE_CLOSED);
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD || gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CLOSE_PIDF);
            shooter.setVelocity(CLOSE_VELOCITY);
            hood.setPosition(HOOD_CLOSE);
        } else if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            shooter.setPower(SHOOTER_REVERSE_POWER);
            hood.setPosition(HOOD_CLOSE);
            gate.setPosition(GATE_CLOSED);
        }
    }

    /** Opens the gate and runs the intake to push artifacts into the shooter. */
    private void feedShooter() {
        gate.setPosition(GATE_OPEN);
        intake.start();
    }

    private void showAimHint() {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addLine(VisionUtil.aimHint(result.getTx()));
            telemetry.addData("Tx", result.getTx());
        }
    }
}
