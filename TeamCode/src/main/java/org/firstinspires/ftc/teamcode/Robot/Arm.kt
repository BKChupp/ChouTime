package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Arm(hardwareMap: HardwareMap) {
    private val claw = hardwareMap.servo.get("claw").apply {
        direction = Servo.Direction.FORWARD
    }
    private var position: Position = Position.INTAKE

    init {
        setPosition(position)
    }

    enum class Position(val position: Double) {
        INTAKE(0.35),
        HOVER(0.43),
        GRAB_WALL(0.95),
        SCORE(0.42),
        LINE_UP(0.98),
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