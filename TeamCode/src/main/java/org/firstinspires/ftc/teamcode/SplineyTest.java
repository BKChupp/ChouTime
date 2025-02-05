package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ActionHardware;



import java.lang.Math;
import java.util.Timer;


@Autonomous
public class SplineyTest extends OpMode {


    private DcMotor pivot;

    private static final double CLAW_OPEN = 0.5;
    private static final double CLAW_CLOSED = 0.29;
    private static final double ARM_READY = 0.42;
    private static final double ARM_DOWN = 0.37;
    private static final double ARM_SCORE = 0.42;
    private static final double ARM_WALL = 0.95;
    private static final double WRIST_READY = 0.94;
    private static final double WRIST_DOWN = 0.90;
    private static final double WRIST_SCORE = 0.96;
    private static final double WRIST_WALL = 0.85;
    private static final double ROTATE_READY = 0.22;
    private static final double ROTATE_SCORE = 0.78;
    private static final double ROTATE_WALL = 0.22;

    private static final int SLIDE1_SCORING_POSITION = 1500;
    private static final int SLIDE_MAX_POSITION = 1400;
    private static final int SLIDE_INITIAL_POSITION = 0; // Starting (retracted) position


    // Pivot Control
    private static final double PIVOT_UP_POWER = 0.8;
    private static final double PIVOT_DOWN_POWER = -0.3;

    private static final long MOVE_TO_SCORE_TIMEOUT = 5000; // 5 seconds


    // Manual Rotate Increment (for Gamepad2 right stick)
    private static final double ROTATE_MANUAL_INCREMENT = 0.01;
    private double rotateManualPosition = ROTATE_READY;


    // Tolerance for checking slide1's ready position
    private static final int SLIDE_READY_TARGET = 400;
    private static final int SLIDE_READY_TOLERANCE = 50;


    // Tolerance for slide retraction (how close to SLIDE_INITIAL_POSITION to consider retracted)
    private static final int SLIDE_RETRACT_TOLERANCE = 5;

    private boolean isRetracting = false;


    // -----------------------
    // State Enums
    // -----------------------
    private enum GrabState { IDLE, INIT, MOVE_ARM_AND_CLAW, RETURN_TO_READY }
    private enum ScoreState { IDLE, INIT, CLOSE_CLAW, MOVE_ARM, ROTATE_SERVO, LIFT_WRIST, MOVE_TO_SCORE }
    private enum PivotState { IDLE, MOVING_UP, MOVING_DOWN }


    // -----------------------
    // State Variables
    // -----------------------
    private GrabState grabState = GrabState.IDLE;
    private ScoreState scoreState = ScoreState.IDLE;
    private PivotState pivotState = PivotState.IDLE;


    private long actionStartTime = 0; // Generic start time for sequences


    // -----------------------
    // Button Press Tracking
    // -----------------------
    private boolean previousGamepad2LeftBumper = false; // For edge detection
    private boolean previousGamepad2RightTrigger = false;


    private DcMotor slide1;
    private Servo claw;
    private Servo arm;
    private Servo rotate;
    private Servo wrist;

    private ElapsedTime     time = new ElapsedTime();

    @Override
    public void init() {

        Pose2d beginPose = new Pose2d(0.0, -62.75, Math.toRadians(90.0));

        slide1 = hardwareMap.dcMotor.get("slide1");
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setDirection(DcMotorSimple.Direction.FORWARD);


        pivot = hardwareMap.dcMotor.get("pivot");
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.servo.get("arm");
        rotate = hardwareMap.servo.get("rotate");
        wrist = hardwareMap.servo.get("wrist");


    }


    @Override
    public void loop() {
        Pose2d beginPose = new Pose2d(0.0, -62.75, Math.toRadians(90.0));


        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        final Action Score = drive
                .actionBuilder(beginPose)
                        // score specimen
                        .waitSeconds(1.8)
                        .lineToY(-39)
                        .lineToY(-52)
                        .build();

        final Action Score2 = drive
                .actionBuilder(beginPose)
                // score specimen
                .strafeTo(new Vector2d(12, -60))
                .waitSeconds(2)
                .lineToY(-46)
                .build();

        final Action GrabSpec = drive
                .actionBuilder(beginPose)
                // score specimen
                .strafeTo(new Vector2d(0.0, -52.0))
                .build();


        final Action PushSpecs = drive
                .actionBuilder(beginPose)
                // push all of the specimens to the human players area
                .strafeTo(new Vector2d(56, -60))
                .build();

        final Action Grab = drive
                .actionBuilder(beginPose)
                .lineToY(-62.0)
                .build();

        final Action Park = drive
                .actionBuilder(beginPose)
                .strafeTo(new Vector2d(37, -55))
                .waitSeconds(15)
                .build();



        claw.setPosition(CLAW_CLOSED);
        rotate.setPosition(ROTATE_SCORE);
        arm.setPosition(ARM_SCORE);
        wrist.setPosition(WRIST_SCORE);

        time.reset();
        while (time.seconds() > 0 && time.seconds() < 1.0) {

            pivot.setPower(0.8);

        }
        pivot.setPower(0.0);

        slide1.setTargetPosition(2200);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(1.0);

        sleep(200);

        Actions.runBlocking(Score);

        claw.setPosition(CLAW_OPEN);
        arm.setPosition(0.45);

        slide1.setTargetPosition(10);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(1.0);

        time.reset();
        while (time.seconds() > 0 && time.seconds() < 0.75) {

            pivot.setPower(-0.45);

        }pivot.setPower(0.0);

       Actions.runBlocking(PushSpecs);


//       claw.setPosition(0.5);
//
//        sleep(150);
//        arm.setPosition(0.95);
//        wrist.setPosition(0.85);
//        rotate.setPosition(0.22);
//
//
//        Actions.runBlocking(Grab);
//
//
//        claw.setPosition(0.29);
//
//       slide1.setTargetPosition(2200);
//       slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//       slide1.setPower(1.0);
//
//       Actions.runBlocking(Score2);
//
//       claw.setPosition(0.5);
//
//       Actions.runBlocking(PushSpecs);


//        sleep(100);
//
//        slide1.setTargetPosition(2200);
//        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slide1.setPower(.75);
//
////       Actions.runBlocking(GrabSpec);
//
//       claw.setPosition(0.29);
//       arm.setPosition(ARM_WALL);
//       wrist.setPosition(WRIST_WALL);
//       rotate.setPosition(ROTATE_WALL);
//
//        Actions.runBlocking(Grab);
//
//        sleep(100);
//
//        claw.setPosition(CLAW_CLOSED);
//
//        sleep(100);
//
//       Actions.runBlocking(Score);
//
//       claw.setPosition(CLAW_OPEN);
//
//        slide1.setTargetPosition(10);
//        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slide1.setPower(1.0);
//
//        sleep(1000);

//        Actions.runBlocking(Park);

//        time.reset();
//        while (time.seconds() > 0 && time.seconds() < 0.75) {
//
//            pivot.setPower(-0.5);
//
//        }pivot.setPower(0.0);
//
//        slide1.setTargetPosition(10);
//        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slide1.setPower(1.0);
//
        sleep(10000);

    }


}



