package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Rotate(hardwareMap: HardwareMap) {
    private val claw = hardwareMap.servo.get("claw").apply {
        direction = Servo.Direction.FORWARD
    }
    private var position: Position = Position.YES

    init {
        setPosition(position)
    }

    enum class Position(val position: Double) {
        YES(0.78),
    }

    fun setPosition(position: Position) {
        this.position = position
        claw.position = position.position
    }

    fun getRawPosition(): Double {
        return claw.position
    }

    fun setRawPosition(position: Double) {
        claw.position = position
    }
}