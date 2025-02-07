package org.firstinspires.ftc.teamcode.Actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.Robot.Wrist

class WristAction(
    private val wrist: Wrist,
    private val wristPosition: Wrist.Position,
) : Action {

    override fun run(p: TelemetryPacket): Boolean {
        wrist.setPosition(wristPosition)
        return false
    }

}