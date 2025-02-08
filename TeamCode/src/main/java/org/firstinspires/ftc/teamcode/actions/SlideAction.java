package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.robot.Slides;

public class SlideAction implements Action {
    private final Slides slide;
    private final int position;

    public SlideAction(Slides slide, int position) {
        this.slide = slide;
        this.position = position;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        slide.setPosition(position);
        return false;
    }
}
