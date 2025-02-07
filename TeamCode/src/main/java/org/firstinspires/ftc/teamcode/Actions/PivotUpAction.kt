package org.firstinspires.ftc.teamcode.Actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap

class PivotUpAction(private val hardwareMap: HardwareMap) : Action {
    private val pivot = hardwareMap.dcMotor.get("pivot")
    private var started: Long? = null

    override fun run(p: TelemetryPacket): Boolean {
        if (started == null) {
            started = System.currentTimeMillis()
            pivot.power = 0.8
            return true
        }

        return System.currentTimeMillis() - started!! <= 1300
    }


}