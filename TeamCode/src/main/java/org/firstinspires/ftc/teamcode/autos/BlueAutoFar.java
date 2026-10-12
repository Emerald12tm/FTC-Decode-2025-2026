package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue alliance poses for {@link FarAuto}. Coordinates are relative to the starting spot. */
@Autonomous(name = "Blue Auto Far", group = "Blue")
public class BlueAutoFar extends FarAuto {

    public BlueAutoFar() {
        startPose = new Pose(0, 0, Math.toRadians(0));
        shootPose = new Pose(3, 3, Math.toRadians(20));
        setupIntakePose = new Pose(3, 20, Math.toRadians(0));
    }
}
