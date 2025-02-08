package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private final DcMotor slide;

    public Slides(HardwareMap hardwareMap) {
        slide = hardwareMap.dcMotor.get("slide1");
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getRawPosition() {
        return slide.getCurrentPosition();
    }

    public void setPower(double power) {
        if (slide.getCurrentPosition() < -1638 && power < 0) {
            slide.setPower(0.0);
        } else if (slide.getCurrentPosition() > 0 && power > 0) {
            slide.setPower(0.0);
        } else {
            slide.setPower(power);
        }
    }

    public void setPosition(int position) {
        slide.setTargetPosition(position);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1.0);
    }
}
