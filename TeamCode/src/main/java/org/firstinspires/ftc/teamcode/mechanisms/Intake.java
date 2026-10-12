package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Intake roller plus the transfer that feeds artifacts up to the shooter. */
public class Intake {

    // Powers that pull artifacts into the robot
    private static final double INTAKE_IN_POWER = -1;
    private static final double TRANSFER_IN_POWER = 1;

    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        stop();
    }

    /** Pulls artifacts in and feeds them toward the shooter. */
    public void start() {
        setPower(INTAKE_IN_POWER, TRANSFER_IN_POWER);
    }

    /** Pushes artifacts back out of the robot. */
    public void reverse() {
        setPower(-INTAKE_IN_POWER, -TRANSFER_IN_POWER);
    }

    public void stop() {
        setPower(0, 0);
    }

    private void setPower(double intakePower, double transferPower) {
        intakeMotor.setPower(intakePower);
        transferMotor.setPower(transferPower);
    }
}
