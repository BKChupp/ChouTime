package org.firstinspires.ftc.teamcode.Actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

class WaitUntilAction(private val op: () -> Boolean) : Action {

    override fun run(p: TelemetryPacket): Boolean {
        return op()
    }

}
