package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.robot.Wrist;

public class WristAction implements Action {
    private final Wrist wrist;
    private final Wrist.Position wristPosition;

    public WristAction(Wrist wrist, Wrist.Position wristPosition) {
        this.wrist = wrist;
        this.wristPosition = wristPosition;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        wrist.setPosition(wristPosition);
        return false;
    }
}
