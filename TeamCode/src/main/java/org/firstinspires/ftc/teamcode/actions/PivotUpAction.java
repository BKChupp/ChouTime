package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotUpAction implements Action {
    private final DcMotor pivot;
    private Long started;

    public PivotUpAction(HardwareMap hardwareMap) {
        this.pivot = hardwareMap.dcMotor.get("pivot");
        this.started = null;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        if (started == null) {
            started = System.currentTimeMillis();
            pivot.setPower(0.8);
            return true;
        }

        return System.currentTimeMillis() - started <= 1300;
    }
}
