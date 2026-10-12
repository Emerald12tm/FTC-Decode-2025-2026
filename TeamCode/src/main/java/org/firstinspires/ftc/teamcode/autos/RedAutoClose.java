package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Red alliance poses for {@link CloseAuto}. */
@Autonomous(name = "Red Auto Close", group = "Red")
public class RedAutoClose extends CloseAuto {

    public RedAutoClose() {
        startPose = new Pose(121, 124, Math.toRadians(-142));
        preloadShootPose = new Pose(70, 100, Math.toRadians(45));
        cycleShootPose = new Pose(70, 100, Math.toRadians(40));
        setupIntake1Pose = new Pose(85, 82, Math.toRadians(0));
        intake1Pose = new Pose(125, 82, Math.toRadians(0));
        setupGatePose = new Pose(110, 87, Math.toRadians(0));
        gatePose = new Pose(119, 87, Math.toRadians(0));
        gateExitPose = new Pose(75, 87, Math.toRadians(0));
        setupIntake2Pose = new Pose(85, 60, Math.toRadians(0));
        intake2Pose = new Pose(110, 60, Math.toRadians(2));
        setupIntake3Pose = new Pose(85, 15, Math.toRadians(2));
        intake3Pose = new Pose(110, 80, Math.toRadians(2)); // TODO verify: y = 80, but setupIntake3Pose is y = 15
        parkPose = new Pose(75, 100, Math.toRadians(40));
    }
}
