package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.robot.Claw;

public class ClawAction implements Action {
    private final Claw claw;
    private final Claw.Position clawPosition;

    public ClawAction(Claw claw, Claw.Position clawPosition) {
        this.claw = claw;
        this.clawPosition = clawPosition;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        claw.setPosition(clawPosition);
        return false;
    }
}
