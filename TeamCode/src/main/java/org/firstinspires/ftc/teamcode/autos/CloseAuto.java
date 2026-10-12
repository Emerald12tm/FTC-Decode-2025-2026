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
 * Close-start autonomous routine shared by both alliances. Scores the preload, intakes spike 1
 * and opens the gate, then scores and intakes spikes 2 and 3 before parking.
 *
 * <p>Subclasses only set the field poses for their alliance in their constructor.
 */
public abstract class CloseAuto extends OpMode {

    /** Each state is named for what the robot is doing while in it. */
    private enum PathState {
        START,
        DRIVE_TO_PRELOAD_SHOT,
        SHOOT_PRELOAD,
        DRIVE_TO_INTAKE_1,
        INTAKE_1,
        DRIVE_TO_GATE,
        OPEN_GATE,
        BACK_AWAY_FROM_GATE,
        DRIVE_TO_SHOT_2,
        SHOOT_2,
        DRIVE_TO_INTAKE_2,
        INTAKE_2,
        DRIVE_TO_SHOT_3,
        SHOOT_3,
        DRIVE_TO_INTAKE_3,
        INTAKE_3,
        DRIVE_TO_SHOT_4,
        SHOOT_4,
        PARK
    }

    /** Max power while driving through artifacts or pushing the gate. */
    private static final double SLOW_SPEED = 0.4;
    /** How long to hold against the gate so that it fully opens. */
    private static final double GATE_HOLD_SECONDS = 1;

    // Field poses, set by each alliance's subclass
    protected Pose startPose;
    protected Pose preloadShootPose;
    protected Pose cycleShootPose;
    protected Pose setupIntake1Pose;
    protected Pose intake1Pose;
    protected Pose setupGatePose;
    protected Pose gatePose;
    protected Pose gateExitPose;
    protected Pose setupIntake2Pose;
    protected Pose intake2Pose;
    protected Pose setupIntake3Pose;
    protected Pose intake3Pose;
    protected Pose parkPose;

    private final Flywheel shooter = new Flywheel();
    private final Intake intake = new Intake();

    private Follower follower;
    private Timer pathTimer;
    private PathState pathState;

    private PathChain startToPreloadShot, preloadShotToIntake1Setup, intake1Path, intake1ToGateSetup,
            pushGatePath, gateToGateExit, gateExitToShot, shotToIntake2Setup, intake2Path,
            intake2ToShot, shotToIntake3Setup, intake3Path, intake3ToShot, shotToPark;

    /** Each path starts where the previous one ends, so the robot never jumps to a new start. */
    private void buildPaths() {
        startToPreloadShot = line(startPose, preloadShootPose);
        preloadShotToIntake1Setup = line(preloadShootPose, setupIntake1Pose);
        intake1Path = line(setupIntake1Pose, intake1Pose);
        intake1ToGateSetup = line(intake1Pose, setupGatePose);
        pushGatePath = line(setupGatePose, gatePose);
        gateToGateExit = line(gatePose, gateExitPose);
        gateExitToShot = line(gateExitPose, cycleShootPose);
        shotToIntake2Setup = line(cycleShootPose, setupIntake2Pose);
        intake2Path = line(setupIntake2Pose, intake2Pose);
        intake2ToShot = line(intake2Pose, cycleShootPose);
        shotToIntake3Setup = line(cycleShootPose, setupIntake3Pose);
        intake3Path = line(setupIntake3Pose, intake3Pose);
        intake3ToShot = line(intake3Pose, cycleShootPose);
        shotToPark = line(cycleShootPose, parkPose);
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
                shooter.spinUp(ShotType.CLOSE);
                follower.followPath(startToPreloadShot, true);
                setPathState(PathState.DRIVE_TO_PRELOAD_SHOT);
                break;
            case DRIVE_TO_PRELOAD_SHOT:
                if (!follower.isBusy()) {
                    startShooting(PathState.SHOOT_PRELOAD);
                }
                break;
            case SHOOT_PRELOAD:
                if (!shooter.isBusy()) {
                    intake.stop();
                    follower.followPath(preloadShotToIntake1Setup, true);
                    setPathState(PathState.DRIVE_TO_INTAKE_1);
                }
                break;
            case DRIVE_TO_INTAKE_1:
                if (!follower.isBusy()) {
                    intake.start();
                    follower.followPath(intake1Path, SLOW_SPEED, true);
                    setPathState(PathState.INTAKE_1);
                }
                break;
            case INTAKE_1:
                if (!follower.isBusy()) {
                    intake.stop();
                    follower.followPath(intake1ToGateSetup, true);
                    setPathState(PathState.DRIVE_TO_GATE);
                }
                break;
            case DRIVE_TO_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(pushGatePath, SLOW_SPEED, true);
                    setPathState(PathState.OPEN_GATE);
                }
                break;
            case OPEN_GATE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > GATE_HOLD_SECONDS) {
                    follower.followPath(gateToGateExit, true);
                    setPathState(PathState.BACK_AWAY_FROM_GATE);
                }
                break;
            case BACK_AWAY_FROM_GATE:
                if (!follower.isBusy()) {
                    shooter.spinUp(ShotType.CLOSE);
                    follower.followPath(gateExitToShot, true);
                    setPathState(PathState.DRIVE_TO_SHOT_2);
                }
                break;
            case DRIVE_TO_SHOT_2:
                if (!follower.isBusy()) {
                    startShooting(PathState.SHOOT_2);
                }
                break;
            case SHOOT_2:
                if (!shooter.isBusy()) {
                    intake.stop();
                    follower.followPath(shotToIntake2Setup, true);
                    setPathState(PathState.DRIVE_TO_INTAKE_2);
                }
                break;
            case DRIVE_TO_INTAKE_2:
                if (!follower.isBusy()) {
                    intake.start();
                    follower.followPath(intake2Path, SLOW_SPEED, true);
                    setPathState(PathState.INTAKE_2);
                }
                break;
            case INTAKE_2:
                if (!follower.isBusy()) {
                    intake.stop();
                    shooter.spinUp(ShotType.CLOSE);
                    follower.followPath(intake2ToShot, true);
                    setPathState(PathState.DRIVE_TO_SHOT_3);
                }
                break;
            case DRIVE_TO_SHOT_3:
                if (!follower.isBusy()) {
                    startShooting(PathState.SHOOT_3);
                }
                break;
            case SHOOT_3:
                if (!shooter.isBusy()) {
                    intake.stop();
                    follower.followPath(shotToIntake3Setup, true);
                    setPathState(PathState.DRIVE_TO_INTAKE_3);
                }
                break;
            case DRIVE_TO_INTAKE_3:
                if (!follower.isBusy()) {
                    intake.start();
                    follower.followPath(intake3Path, SLOW_SPEED, true);
                    setPathState(PathState.INTAKE_3);
                }
                break;
            case INTAKE_3:
                if (!follower.isBusy()) {
                    intake.stop();
                    shooter.spinUp(ShotType.CLOSE);
                    follower.followPath(intake3ToShot, true);
                    setPathState(PathState.DRIVE_TO_SHOT_4);
                }
                break;
            case DRIVE_TO_SHOT_4:
                if (!follower.isBusy()) {
                    startShooting(PathState.SHOOT_4);
                }
                break;
            case SHOOT_4:
                if (!shooter.isBusy()) {
                    intake.stop();
                    follower.followPath(shotToPark, true);
                    setPathState(PathState.PARK);
                }
                break;
            case PARK:
                // Nothing left to do; the follower finishes the park path on its own
                break;
        }
    }

    /** Requests a shot and runs the intake to feed artifacts through the gate once it opens. */
    private void startShooting(PathState nextState) {
        shooter.fire(ShotType.CLOSE);
        intake.start();
        setPathState(nextState);
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
