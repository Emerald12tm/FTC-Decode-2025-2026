package org.firstinspires.ftc.teamcode.util;

/** Shared helpers for interpreting Limelight results. */
public final class VisionUtil {

    /** How far off center (tx, in degrees) the target can be while still counting as aimed. */
    public static final double AIM_TOLERANCE_DEGREES = 1.5;

    private VisionUtil() {
    }

    /** Estimates distance to the target from its area (ta), using a measured power-curve fit. */
    public static double distanceFromTargetArea(double targetArea) {
        return 187.7246 / Math.pow(targetArea, 0.464831);
    }

    /** Driver hint for which way to turn so that the target is centered. */
    public static String aimHint(double tx) {
        if (Math.abs(tx) < AIM_TOLERANCE_DEGREES) {
            return "READY TO SHOOT";
        }
        return tx > 0 ? "TURN RIGHT" : "TURN LEFT";
    }
}
