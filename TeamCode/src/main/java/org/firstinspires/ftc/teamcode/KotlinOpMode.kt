package org.firstinspires.ftc.teamcode

import androidx.core.graphics.scaleMatrix
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "Kotlin Op Mode")
class KotlinOpMode : LinearOpMode() {

    private lateinit var claw: Servo

    override fun runOpMode() {

        claw = hardwareMap.get(Servo::class.java, "claw")

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        claw.position = 0.28

        while (opModeIsActive()) {

            if (gamepad2.a){
                claw.position = 0.5
            }

            telemetry.addData("Status", "Running")
            telemetry.update()
        }
    }

}
