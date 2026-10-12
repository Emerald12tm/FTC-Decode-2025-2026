package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.VisionUtil;

/** Shows raw Limelight readings and the estimated distance to the target. */
@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addLine(VisionUtil.aimHint(result.getTx()));
            telemetry.addData("Distance", VisionUtil.distanceFromTargetArea(result.getTa()));
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.addData("Botpose", result.getBotpose_MT2());
        }
    }
}
