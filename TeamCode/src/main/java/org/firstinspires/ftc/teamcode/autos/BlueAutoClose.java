package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue alliance poses for {@link CloseAuto}. */
@Autonomous(name = "Blue Auto Close", group = "Blue")
public class BlueAutoClose extends CloseAuto {

    public BlueAutoClose() {
        startPose = new Pose(21, 124, Math.toRadians(-37));
        preloadShootPose = new Pose(30, 92, Math.toRadians(135));
        cycleShootPose = new Pose(40, 75, Math.toRadians(140));
        setupIntake1Pose = new Pose(40, 36, Math.toRadians(182));
        intake1Pose = new Pose(5, 36, Math.toRadians(182));
        setupGatePose = new Pose(15, 43, Math.toRadians(182));
        gatePose = new Pose(10, 43, Math.toRadians(182));
        gateExitPose = new Pose(30, 43, Math.toRadians(170));
        setupIntake2Pose = new Pose(30, 60, Math.toRadians(182));
        intake2Pose = new Pose(10, 60, Math.toRadians(182));
        setupIntake3Pose = new Pose(40, 15, Math.toRadians(182));
        intake3Pose = new Pose(5, 15, Math.toRadians(182));
        parkPose = new Pose(30, 100, Math.toRadians(142));
    }
}
