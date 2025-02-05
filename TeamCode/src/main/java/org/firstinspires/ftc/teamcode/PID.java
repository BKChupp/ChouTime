package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;




@Config
@TeleOp
public class PID extends OpMode {


    private PIDController controller;


    public static double p = 0.038, i = 0.0008, d = 0.0005;
    //then tune these^
    public static double f = 0.29; // tune this first use this video as a reminder youtube.com/watch?v=E6H6Nqe6qJo
    // to tune this^ pick up the arm and see if it resist the force of gravity


    public static double target = 0; // 330 encoder tickes is the limit.


    private final double ticksindegree = 3895.9 / 360;


    private DcMotorEx pivot;
//    private DcMotor slide1;
    private DcMotor slide2;
    private DcMotor slide1;
//    private DcMotorEx pivot2;


    @Override
    public void init() {
        controller = new PIDController(p , i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide2 = hardwareMap.get(DcMotor.class, "slide2");
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        pivot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        pivot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        pivot2.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop() {


        controller.setPID(p, i ,d);
        int pivot1Pos = pivot.getCurrentPosition();




        double pid1 = controller.calculate(pivot1Pos, target);


        double ff = Math.cos(Math.toRadians(target / ticksindegree)) * f;


        double power = pid1 + ff;


        pivot.setPower(power);
        if(gamepad1.a){
            target = 670;
        }
//        pivot2.setPower(-power);

        slide1.setPower(-gamepad2.left_stick_y);
        slide2.setPower(-gamepad2.left_stick_y);

        telemetry.addData("pivot1 position", pivot1Pos);


        telemetry.addData("target", target);
        telemetry.update();
    }
}
