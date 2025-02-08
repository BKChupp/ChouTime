package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.robot.Rotate;

public class RotateAction implements Action {
    private final Rotate rotate;
    private final Rotate.Position rotatePosition;

    public RotateAction(Rotate rotate, Rotate.Position rotatePosition) {
        this.rotate = rotate;
        this.rotatePosition = rotatePosition;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        rotate.setPosition(rotatePosition);
        return false;
    }
}
