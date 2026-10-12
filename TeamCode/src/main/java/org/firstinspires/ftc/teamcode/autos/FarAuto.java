package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel.ShotType;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Far-start autonomous routine shared by both alliances. Scores the preload with a far shot, then
 * drives to the first spike mark and runs the intake.
 *
 * <p>Subclasses only set the field poses for their alliance in their constructor.
 */
public abstract class FarAuto extends OpMode {

    /** Each state is named for what the robot is doing while in it. */
    private enum PathState {
        START,
        DRIVE_TO_SHOT,
        SHOOT_PRELOAD,
        DRIVE_TO_INTAKE,
        DONE
    }

    // Field poses, set by each alliance's subclass
    protected Pose startPose;
    protected Pose shootPose;
    protected Pose setupIntakePose;

    private final Flywheel shooter = new Flywheel();
    private final Intake intake = new Intake();

    private Follower follower;
    private Timer pathTimer;
    private PathState pathState;

    private PathChain startToShot, shotToIntakeSetup;

    private void buildPaths() {
        startToShot = line(startPose, shootPose);
        shotToIntakeSetup = line(shootPose, setupIntakePose);
    }

    private PathChain line(Pose from, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    private void updatePathState() {
        switch (pathState) {
            case START:
                shooter.spinUp(ShotType.FAR);
                follower.followPath(startToShot, true);
                setPathState(PathState.DRIVE_TO_SHOT);
                break;
            case DRIVE_TO_SHOT:
                if (!follower.isBusy()) {
                    shooter.fire(ShotType.FAR);
                    intake.start();
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case SHOOT_PRELOAD:
                if (!shooter.isBusy()) {
                    intake.stop();
                    follower.followPath(shotToIntakeSetup, true);
                    setPathState(PathState.DRIVE_TO_INTAKE);
                }
                break;
            case DRIVE_TO_INTAKE:
                if (!follower.isBusy()) {
                    intake.start();
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                break;
        }
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        shooter.init(hardwareMap);
        intake.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
        pathState = PathState.START;
    }

    @Override
    public void start() {
        setPathState(PathState.START);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        updatePathState();

        telemetry.addData("Path state", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Time in state", pathTimer.getElapsedTimeSeconds());
    }
}
