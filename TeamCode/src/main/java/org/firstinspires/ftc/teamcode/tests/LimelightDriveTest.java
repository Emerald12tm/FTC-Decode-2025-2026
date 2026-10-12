package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.VisionUtil;

/** Drive-only teleop that shows Limelight aim hints and distance while driving. */
@TeleOp(name = "Limelight Drive Test", group = "Test")
public class LimelightDriveTest extends LinearOpMode {

    private static final double HOOD_CLOSE = 0.51;
    private static final double GATE_CLOSED = 1.0;

    private final MecanumDrive drive = new MecanumDrive();
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        // Hold the shooter servos in place while driving
        hardwareMap.get(Servo.class, "hoodServo").setPosition(HOOD_CLOSE);
        hardwareMap.get(Servo.class, "gateServo").setPosition(GATE_CLOSED);

        telemetry.addLine("Init Complete");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addLine(VisionUtil.aimHint(result.getTx()));
                telemetry.addData("Distance", VisionUtil.distanceFromTargetArea(result.getTa()));
                telemetry.addData("Tx", result.getTx());
            }
            telemetry.update();
        }
    }
}
