package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Kotlin Op Mode")
class KotlinOpMode : LinearOpMode() {

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running")
            telemetry.update()
        }
    }

}
