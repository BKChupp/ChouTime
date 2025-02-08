package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotDownAction implements Action {
    private final DcMotor pivot;
    private Long started;

    public PivotDownAction(HardwareMap hardwareMap) {
        this.pivot = hardwareMap.dcMotor.get("pivot");
        this.started = null;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        if (started == null) {
            started = System.currentTimeMillis();
            pivot.setPower(-0.3);
            return true;
        }

        return System.currentTimeMillis() - started <= 1000;
    }
}
