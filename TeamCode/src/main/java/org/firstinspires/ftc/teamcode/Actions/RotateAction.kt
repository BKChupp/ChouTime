package org.firstinspires.ftc.teamcode.Actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.Robot.Rotate
import org.firstinspires.ftc.teamcode.Robot.Wrist

class RotateAction(
    private val rotate: Rotate,
    private val rotatePosition: Rotate.Position,
) : Action {

    override fun run(p: TelemetryPacket): Boolean {
        rotate.setPosition(rotatePosition)
        return false
    }

}