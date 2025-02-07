package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class Pivot(hardwareMap: HardwareMap) {

    //    var targetPosition: Int
    private val pivot = hardwareMap.dcMotor.get("pivot").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

}