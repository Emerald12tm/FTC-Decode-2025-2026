package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Red alliance poses for {@link FarAuto}. Coordinates are relative to the starting spot. */
@Autonomous(name = "Red Auto Far", group = "Red")
public class RedAutoFar extends FarAuto {

    public RedAutoFar() {
        startPose = new Pose(0, 0, Math.toRadians(0));
        shootPose = new Pose(3, 3, Math.toRadians(-20));
        setupIntakePose = new Pose(3, -20, Math.toRadians(0));
    }
}
