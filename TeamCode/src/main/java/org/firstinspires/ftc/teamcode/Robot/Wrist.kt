package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.robotcore.hardware.HardwareMap

class Wrist(hardwareMap: HardwareMap) {
    private val wrist = hardwareMap.servo.get("wrist")

    private var position: Position = Position.INTAKE

    init {
        setPosition(position)
    }

    enum class Position(val position: Double) {
        INTAKE(0.9),
        HOVER(0.94),
        GRAB_WALL(0.85),
        SCORE(0.94),
        LINE_UP(0.75),
    }

    fun setPosition(position: Position) {
        this.position = position
        wrist.position = position.position
    }

    fun setRawPosition(position: Double) {
        wrist.position = position
    }

    fun getRawPosition(): Double {
        return wrist.position
    }

    fun getPosition(): Position {
        return position
    }
}
