package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class DelayAction implements Action {
    private final long delayMs;
    private Long startTime;

    public DelayAction(long delayMs) {
        this.delayMs = delayMs;
        this.startTime = null;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        if (startTime == null) {
            startTime = System.currentTimeMillis();
            return true;
        } else {
            return System.currentTimeMillis() - startTime < delayMs;
        }
    }
}
