package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ActionHardware {


    private static final double TRIGGER_THRESHOLD = 0.1;


    // Servo Positions
    private static final double CLAW_OPEN = 0.5;
    private static final double CLAW_CLOSED = 0.29;
    private static final double ARM_READY = 0.42;
    private static final double ARM_DOWN = 0.37;
    private static final double ARM_SCORE = 0.4;
    private static final double ARM_WALL = 0.95;
    private static final double WRIST_READY = 0.94;
    private static final double WRIST_DOWN = 0.90;
    private static final double WRIST_SCORE = 0.96;
    private static final double WRIST_WALL = 0.85;
    private static final double ROTATE_READY = 0.22;
    private static final double ROTATE_SCORE = 0.78;
    private static final double ROTATE_WALL = 0.22;


    // Slide Positions
    private static final int SLIDE1_SCORING_POSITION = 1500;
    private static final int SLIDE_MAX_POSITION = 1400;
    private static final int SLIDE_INITIAL_POSITION = 0; // Starting (retracted) position


    // Pivot Control
    private static final double PIVOT_UP_POWER = 0.8;
    private static final double PIVOT_DOWN_POWER = -0.3;


    // Timeouts (in milliseconds)
    private static final long MOVE_TO_SCORE_TIMEOUT = 5000; // 5 seconds


    // Manual Rotate Increment (for Gamepad2 right stick)
    private static final double ROTATE_MANUAL_INCREMENT = 0.01;
    private double rotateManualPosition = ROTATE_READY;


    // Tolerance for checking slide1's ready position
    private static final int SLIDE_READY_TARGET = 400;
    private static final int SLIDE_READY_TOLERANCE = 50;


    // Tolerance for slide retraction (how close to SLIDE_INITIAL_POSITION to consider retracted)
    private static final int SLIDE_RETRACT_TOLERANCE = 5;


    // -----------------------
    // New Flag for Slide Retraction
    // -----------------------
    private boolean isRetracting = false;


    // -----------------------
    // State Enums
    // -----------------------
    private enum GrabState { IDLE, INIT, MOVE_ARM_AND_CLAW, RETURN_TO_READY }
    private enum ScoreState { IDLE, INIT, CLOSE_CLAW, MOVE_ARM, ROTATE_SERVO, LIFT_WRIST, MOVE_TO_SCORE }
    enum PivotState { IDLE, MOVING_UP, MOVING_DOWN }


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
    private DcMotor pivot;
    private Servo claw;
    private Servo arm;
    private Servo rotate;
    private Servo wrist;


    public ActionHardware(HardwareMap hardwareMap) {
        slide1 = hardwareMap.dcMotor.get("Slide1");
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


    private boolean isPivotUp() {
        final int PIVOT_UP_THRESHOLD = 100;
        return pivot.getCurrentPosition() > PIVOT_UP_THRESHOLD;
    }




    private void setServoPositions(double armPos, double clawPos, double wristPos, double rotatePos) {
        arm.setPosition(armPos);
        claw.setPosition(clawPos);
        wrist.setPosition(wristPos);
        rotate.setPosition(rotatePos);
    }


    public Action setWallPosition() {
        setServoPositions(ARM_WALL, CLAW_CLOSED, WRIST_WALL, ROTATE_WALL);
        if (!isPivotUp()) {
            startPivotUp();
        }
        isRetracting = true;
        slide1.setTargetPosition(SLIDE_INITIAL_POSITION);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(1.0);
        return null;
    }

    private void startPivotUp() {
        if (pivotState == PivotState.IDLE) {
            pivot.setPower(PIVOT_UP_POWER);
            pivotState = PivotState.MOVING_UP;
            actionStartTime = System.currentTimeMillis();
        }
    }


}






