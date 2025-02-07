package org.firstinspires.ftc.teamcode.Actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.Robot.Arm
import org.firstinspires.ftc.teamcode.Robot.Wrist

class ArmAction(
    private val arm: Arm,
    private val armPosition: Arm.Position,
) : Action {

    override fun run(p: TelemetryPacket): Boolean {
        arm.setPosition(armPosition)
        return false
    }

}