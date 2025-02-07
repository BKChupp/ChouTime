package org.firstinspires.ftc.teamcode.Actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.Robot.Slides

class SlideAction(
    private val slide: Slides,
    private val position: Int,
) : Action {

    override fun run(p: TelemetryPacket): Boolean {
        slide.setPosition(position)
        return false
    }
}