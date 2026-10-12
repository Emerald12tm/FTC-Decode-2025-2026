package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter flywheel with a hood and a gate servo, used by the autos.
 *
 * <p>Call {@link #spinUp(ShotType)} early (e.g. while driving to the shooting spot) so the wheel
 * is already at speed on arrival, then {@link #fire(ShotType)} once in position. The gate only
 * opens after a fire request <em>and</em> once the wheel is at speed. It stays open for
 * {@link #GATE_OPEN_SECONDS}, then the wheel shuts off. Call {@link #update()} every loop.
 */
public class Flywheel {

    /** Which shot to take. This sets the flywheel velocity and hood angle. */
    public enum ShotType {
        CLOSE,
        FAR
    }

    private enum State {
        IDLE,
        SPINNING_UP,
        AT_SPEED,
        FIRING
    }

    private static final double GATE_CLOSED = 1.0;
    private static final double GATE_OPEN = 0.4;

    private static final double HOOD_CLOSE = 0.3;
    private static final double HOOD_FAR = 0.1;

    // Velocities are in encoder ticks per second
    private static final double CLOSE_VELOCITY = 1200;
    private static final double FAR_VELOCITY = 1800;

    /** Fraction of the target velocity that counts as "at speed". */
    private static final double READY_FRACTION = 0.95;
    /** Treat the wheel as at speed after this long, even if it never reaches the target. */
    private static final double MAX_SPINUP_SECONDS = 2;
    private static final double GATE_OPEN_SECONDS = 2;

    private static final double P = 130;
    private static final double F = 13.154;

    private DcMotorEx shooter;
    private Servo gateServo;
    private Servo hoodServo;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private State state = State.IDLE;
    private double targetVelocity = 0;
    private boolean fireRequested = false;

    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
        stop();
    }

    public void update() {
        switch (state) {
            case IDLE:
                break;
            case SPINNING_UP:
                if (shooter.getVelocity() >= targetVelocity * READY_FRACTION
                        || stateTimer.seconds() > MAX_SPINUP_SECONDS) {
                    state = State.AT_SPEED;
                }
                break;
            case AT_SPEED:
                if (fireRequested) {
                    fireRequested = false;
                    gateServo.setPosition(GATE_OPEN);
                    stateTimer.reset();
                    state = State.FIRING;
                }
                break;
            case FIRING:
                if (stateTimer.seconds() > GATE_OPEN_SECONDS) {
                    stop();
                }
                break;
        }
    }

    /**
     * Starts the wheel and sets the hood without opening the gate. Calling it again with the same
     * shot type is harmless. Ignored while a shot is being fired.
     */
    public void spinUp(ShotType shotType) {
        if (state == State.FIRING) {
            return;
        }
        double velocity = (shotType == ShotType.FAR) ? FAR_VELOCITY : CLOSE_VELOCITY;
        if (state == State.IDLE || velocity != targetVelocity) {
            targetVelocity = velocity;
            shooter.setVelocity(velocity);
            hoodServo.setPosition(shotType == ShotType.FAR ? HOOD_FAR : HOOD_CLOSE);
            gateServo.setPosition(GATE_CLOSED);
            stateTimer.reset();
            state = State.SPINNING_UP;
        }
    }

    /** Requests a shot. This spins up first if needed, and is ignored while already firing. */
    public void fire(ShotType shotType) {
        if (state == State.FIRING) {
            return;
        }
        spinUp(shotType);
        fireRequested = true;
    }

    /** True from the moment a shot is requested until the gate closes again. */
    public boolean isBusy() {
        return fireRequested || state == State.FIRING;
    }

    private void stop() {
        shooter.setVelocity(0);
        hoodServo.setPosition(HOOD_CLOSE);
        gateServo.setPosition(GATE_CLOSED);
        targetVelocity = 0;
        fireRequested = false;
        state = State.IDLE;
    }
}
