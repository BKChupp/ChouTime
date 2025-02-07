package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Simplified TeleOp Mode for FTC Robot
 *
 * Features:
 * - Basic Mecanum Drive
 * - Pivot Control (Light Movement on Gamepad1 Left Bumper)
 * - Claw/Arm/Wrist/Rotate Servos with automated sequences:
 *     1) Grabbing (Right Trigger on Gamepad2)
 *     2) Wall Scoring (Left Bumper on Gamepad2) - Moves servos to wall positions,
 *        closes claw, then moves the servos to scoring positions and commands the slide to extend.
 *        This sequence only works if the pivot is already up.
 * - Preset Positions:
 *     - Wall: Right Bumper on Gamepad2
 *         * When activated, the pivot is commanded to go up (if not already)
 *           and the slide is retracted (intaked) to its starting position and then made limp.
 *     - Ready: Automatically activated when slide1 is near 300 and the pivot is fully down.
 * - Manual Slide and Rotate Controls (Gamepad2 Left and Right Sticks)
 *   * Only active when the pivot is fully down and the slide is not retracting.
 * - Claw Open Control:
 *     * Claw opens when left trigger on Gamepad2 is pressed.
 * - State Management
 */
@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends OpMode {

    // -----------------------
    // Hardware Components
    // -----------------------
    private DcMotor fl, fr, bl, br;         // Mecanum drive motors
    private DcMotor pivot;                  // Pivot motor
    private DcMotor slide1;                 // Slide motor

    private Servo claw, arm, rotate, wrist;  // Servos

    // -----------------------
    // Constants and Configurations
    // -----------------------
    private static final double TRIGGER_THRESHOLD = 0.1;

    // Servo Positions
    private static final double CLAW_OPEN      = 0.5;
    private static final double CLAW_CLOSED    = 0.27;
    private static final double ARM_READY      = 0.43;
    private static final double ARM_DOWN       = 0.35;
    private static final double ARM_SCORE      = 0.42;  // Scoring position for arm
    private static final double ARM_WALL       = 0.95;  // Wall position for arm
    private static final double ARM_PRE        = 0.98;
    private static final double WRIST_READY    = 0.94;
    private static final double WRIST_DOWN     = 0.90;
    private static final double WRIST_SCORE    = 0.94;  // Scoring position for wrist
    private static final double WRIST_WALL     = 0.85;  // Wall position for wrist
    private static final double WRIST_PRE      = 0.75;
    private static final double ROTATE_READY   = 0.78;
    private static final double ROTATE_SCORE   = 0.78;  // Scoring position for rotate
    private static final double ROTATE_WALL    = 0.78;  // Wall position for rotate
    private static final double ROTATE_PRE     = 0.78;  // .78 IS FLIPPED

    // Slide Positions
    private static final int SLIDE1_SCORING_POSITION = 1450;
    private static final int SLIDE_MAX_POSITION       = 1100;
    private static final int SLIDE_INITIAL_POSITION   = 0;  // Starting (retracted) position

    // Pivot Control
    private static final double PIVOT_UP_POWER    = 0.8;
    private static final double PIVOT_DOWN_POWER  = -0.3;

    // Timeouts (in milliseconds)
    private static final long MOVE_TO_SCORE_TIMEOUT = 5000; // 5 seconds

    // Manual Rotate Increment (for Gamepad2 right stick)
    private static final double ROTATE_MANUAL_INCREMENT = 0.04;
    private double rotateManualPosition = ROTATE_READY;

    // Tolerance for checking slide1's ready position
    private static final int SLIDE_READY_TARGET = 300;
    private static final int SLIDE_READY_TOLERANCE = 50;

    // Tolerance for slide retraction (how close to SLIDE_INITIAL_POSITION to consider retracted)
    private static final int SLIDE_RETRACT_TOLERANCE = 5;

    // -----------------------
    // New Flag for Slide Retraction
    // -----------------------
    private boolean isRetracting = false;

    // -----------------------
    // New Variables for Wall Sequence Delay
    // -----------------------
    private boolean wallSequenceInProgress = false;
    private long wallSequenceStartTime = 0;

    // -----------------------
    // State Enums
    // -----------------------
    private enum GrabState { IDLE, INIT, MOVE_ARM_AND_CLAW, RETURN_TO_READY }
    // Updated ScoreState for wall scoring sequence:
    private enum ScoreState { IDLE, INIT_WALL, WAIT_FOR_SERVOS, CLOSE_CLAW, WAIT_FOR_CLAW, MOVE_TO_SCORE }
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
    private boolean previousGamepad2RightTrigger = false; // For edge detection

    // -----------------------
    // Initialization
    // -----------------------
    @Override
    public void init() {
        // Initialize Drive Motors
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // Set Drive Motor Directions for Mecanum
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // Initialize Pivot Motor
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Slide1 Motor
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setDirection(DcMotor.Direction.FORWARD);

        // Initialize Servos
        claw = hardwareMap.get(Servo.class, "claw");
        arm  = hardwareMap.get(Servo.class, "arm");
        rotate = hardwareMap.get(Servo.class, "rotate");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        // Optionally, set initial servo positions here if desired.
        // setServoPositions(ARM_READY, CLAW_OPEN, WRIST_READY, ROTATE_READY);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Sets all servos to specified positions.
     */
    private void setServoPositions(double armPos, double clawPos, double wristPos, double rotatePos) {
        arm.setPosition(armPos);
        claw.setPosition(clawPos);
        wrist.setPosition(wristPos);
        rotate.setPosition(rotatePos);
    }

    // -----------------------
    // Main Loop
    // -----------------------
    @Override
    public void loop() {
        handleDrive();

        // New: Check for wrist reset using Gamepad1 Right Bumper.
        handleWristReset();

        // Manual controls are enabled only when:
        // - No automated sequence is running,
        // - The pivot is fully down (using our updated threshold),
        // - The slide is not currently retracting.
        if (grabState == GrabState.IDLE && scoreState == ScoreState.IDLE &&
                isPivotFullyDown() && !isRetracting) {
            handleManualSlideControl();
            handleManualRotateServo();
        }

        // Claw Open Control (Gamepad2 Left Trigger)
        handleClawOpen();

        handlePresetPositions();
        handleSequences();
        handlePivotControl();
        updateSlideRetraction();

        // Check for wall sequence delay (if triggered via the preset wall sequence)
        if (wallSequenceInProgress) {
            long now = System.currentTimeMillis();
            if (now - wallSequenceStartTime >= 200) {
                if (!isPivotUp()) {
                    startPivotUp();
                }
                wallSequenceInProgress = false;
                telemetry.addData("Wall Sequence", "Pivot up started after delay");
                telemetry.update();
            }
        }

        // Auto Ready: Only trigger when in manual mode (i.e. slide is not retracting)
        if (grabState == GrabState.IDLE && scoreState == ScoreState.IDLE &&
                isPivotFullyDown() && !isRetracting) {
            int slidePos = slide1.getCurrentPosition();
            telemetry.addData("SlidePos", slidePos); // Debug telemetry
            if (Math.abs(slidePos - SLIDE_READY_TARGET) < SLIDE_READY_TOLERANCE) {
                setReadyPosition();
            }
        }

        sendTelemetry();
    }

    /**
     * Handles resetting the wrist to its ready position when Gamepad1 Right Bumper is pressed.
     */
    private void handleWristReset() {
        if (gamepad1.right_bumper) {
            wrist.setPosition(WRIST_READY);
            telemetry.addData("Wrist Reset", "Wrist set to ready position via Gamepad1 Right Bumper");
        }
    }

    // -----------------------
    // Drive Control
    // -----------------------
    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotatePower = gamepad1.right_stick_x;

        double flPower = forward + strafe + rotatePower;
        double frPower = forward - strafe - rotatePower;
        double blPower = forward - strafe + rotatePower;
        double brPower = forward + strafe - rotatePower;

        // Normalize motor powers
        double maxPower = Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower),
                        Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    // -----------------------
    // Manual Slide Control (Gamepad2 Left Stick)
    // -----------------------
    private void handleManualSlideControl() {
        double slideInput = -gamepad2.left_stick_y; // Positive extends the slide
        int currentSlide1Pos = slide1.getCurrentPosition();
        if (currentSlide1Pos >= SLIDE_MAX_POSITION && slideInput > 0) {
            slideInput = 0;
        }
        if (currentSlide1Pos <= SLIDE_INITIAL_POSITION && slideInput < 0) {
            slideInput = 0;
        }
        // Use RUN_WITHOUT_ENCODER mode for manual control.
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setPower(slideInput);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -----------------------
    // Manual Rotate Servo Control (Gamepad2 Right Stick)
    // -----------------------
    private void handleManualRotateServo() {
        double input = gamepad2.right_stick_x;
        if (Math.abs(input) > 0.1) {
            rotateManualPosition += input * ROTATE_MANUAL_INCREMENT;
            if (rotateManualPosition > 1) {
                rotateManualPosition = 1;
            } else if (rotateManualPosition < 0) {
                rotateManualPosition = 0;
            }
            rotate.setPosition(rotateManualPosition);
        }
    }

    // -----------------------
    // Claw Open Control (Gamepad2 Left Trigger)
    // -----------------------
    private void handleClawOpen() {
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            claw.setPosition(CLAW_OPEN);
            telemetry.addData("Claw", "Opened");
        }
    }

    // -----------------------
    // Preset Positions Control
    // -----------------------
    private void handlePresetPositions() {
        // Wall Position (Right Bumper on Gamepad2): Retract the slide.
        if (gamepad2.right_bumper && grabState == GrabState.IDLE && scoreState == ScoreState.IDLE) {
            setWallPosition();
        }
        // Pivot control via Gamepad1 Left Bumper remains.
        handlePivotButtonPress();
    }

    /**
     * Sets the robot to the Ready Position.
     */
    private void setReadyPosition() {
        setServoPositions(ARM_READY, CLAW_OPEN, WRIST_READY, ROTATE_READY);
        pivot.setPower(0.0);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Preset Position", "Ready");
        telemetry.update();
    }

    /**
     * Sets the robot to the Wall Position.
     * - Sets servos for wall.
     * - Delays the pivot up command by 1000ms.
     * - Retracts slide1 (intakes it) to its starting position.
     * - Marks the retraction flag so manual slide control is disabled.
     */
    private void setWallPosition() {
        // Set servos immediately. (Using pre positions here; these may be adjusted as needed.)
        setServoPositions(ARM_PRE, CLAW_OPEN, WRIST_PRE, ROTATE_PRE);
        // Record time and mark that the wall sequence is in progress.
        wallSequenceStartTime = System.currentTimeMillis();
        wallSequenceInProgress = true;

        // Begin slide retraction.
        isRetracting = true;
        slide1.setTargetPosition(SLIDE_INITIAL_POSITION);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(1.0);

        telemetry.addData("Preset Position", "Wall (with slide retraction) - Servos set");
        telemetry.update();
    }

    // -----------------------
    // Sequence Handling
    // -----------------------
    private void handleSequences() {
        handleGrabbingSequence();
        handleScoringSequence();
    }

    // -----------------------
    // Grabbing Sequence
    // -----------------------
    private void handleGrabbingSequence() {
        boolean currentRightTriggerPressed = gamepad2.right_trigger > TRIGGER_THRESHOLD;
        if (currentRightTriggerPressed && !previousGamepad2RightTrigger &&
                grabState == GrabState.IDLE && scoreState == ScoreState.IDLE) {
            grabState = GrabState.INIT;
            actionStartTime = System.currentTimeMillis();
        }
        previousGamepad2RightTrigger = currentRightTriggerPressed;
        updateGrabbingSequence();
    }

    private void updateGrabbingSequence() {
        if (grabState == GrabState.IDLE) return;
        long now = System.currentTimeMillis();
        switch (grabState) {
            case INIT:
                arm.setPosition(ARM_DOWN);
                claw.setPosition(CLAW_CLOSED);
                grabState = GrabState.MOVE_ARM_AND_CLAW;
                actionStartTime = now;
                break;
            case MOVE_ARM_AND_CLAW:
                if (now - actionStartTime >= 80) {
                    wrist.setPosition(WRIST_DOWN);
                    grabState = GrabState.RETURN_TO_READY;
                    actionStartTime = now;
                }
                break;
            case RETURN_TO_READY:
                if (now - actionStartTime >= 80) {
                    arm.setPosition(ARM_READY);
                    wrist.setPosition(WRIST_READY);
                    grabState = GrabState.IDLE;
                }
                break;
            default:
                grabState = GrabState.IDLE;
                break;
        }
    }

    // -----------------------
    // Wall Scoring Sequence
    // -----------------------
    /**
     * This method initiates the wall scoring sequence when Gamepad2 left bumper is pressed.
     * The sequence moves the servos to wall positions, closes the claw, then after a short delay
     * moves the servos to the scoring positions and commands the slide to extend.
     * The sequence will only run if the pivot is already up.
     */
    private void handleScoringSequence() {
        // Only allow scoring if the pivot is up.
        if (!isPivotUp()) {
            scoreState = ScoreState.IDLE;
            return;
        }
        boolean currentLeftBumperPressed = gamepad2.left_bumper;
        if (currentLeftBumperPressed && !previousGamepad2LeftBumper &&
                scoreState == ScoreState.IDLE && grabState == GrabState.IDLE) {
            scoreState = ScoreState.INIT_WALL; // Start the new wall scoring sequence.
            actionStartTime = System.currentTimeMillis();
        }
        previousGamepad2LeftBumper = currentLeftBumperPressed;
        updateScoringSequence();
    }

    private void updateScoringSequence() {
        if (scoreState == ScoreState.IDLE) return;
        long now = System.currentTimeMillis();
        switch (scoreState) {
            case INIT_WALL:
                // Set servos to wall positions.
                arm.setPosition(ARM_WALL);
                rotate.setPosition(ROTATE_WALL);
                wrist.setPosition(WRIST_WALL);
                telemetry.addData("Scoring Sequence", "Moving servos to wall positions");
                telemetry.update();
                actionStartTime = now;
                scoreState = ScoreState.WAIT_FOR_SERVOS;
                break;
            case WAIT_FOR_SERVOS:
                // Wait 100 ms for the servos to move.
                if (now - actionStartTime >= 100) {
                    scoreState = ScoreState.CLOSE_CLAW;
                    actionStartTime = now;
                }
                break;
            case CLOSE_CLAW:
                // Now close the claw.
                claw.setPosition(CLAW_CLOSED);
                telemetry.addData("Scoring Sequence", "Claw closing");
                telemetry.update();
                scoreState = ScoreState.WAIT_FOR_CLAW;
                actionStartTime = now;
                break;
            case WAIT_FOR_CLAW:
                // Wait another 100 ms for the claw to finish moving.
                if (now - actionStartTime >= 100) {
                    // Now command the servos to move to their scoring positions.
                    arm.setPosition(ARM_SCORE);
                    wrist.setPosition(WRIST_SCORE);
                    rotate.setPosition(ROTATE_SCORE);
                    telemetry.addData("Scoring Sequence", "Servos moving to scoring positions");
                    telemetry.update();

                    // Ensure that the pivot is up before moving the slide.
                    if (isPivotUp()) {
                        slide1.setTargetPosition(SLIDE1_SCORING_POSITION);
                        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slide1.setPower(1);
                        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        telemetry.addData("Scoring Sequence", "Slide moving to scoring position");
                        telemetry.update();
                        scoreState = ScoreState.MOVE_TO_SCORE;
                        actionStartTime = now;
                    } else {
                        scoreState = ScoreState.IDLE;
                        telemetry.addData("Scoring", "Pivot not up; aborting wall scoring sequence");
                        telemetry.update();
                    }
                }
                break;
            case MOVE_TO_SCORE:
                // Wait until the slide has reached its target (or timeout occurs).
                if (!slide1.isBusy() || (now - actionStartTime) > MOVE_TO_SCORE_TIMEOUT) {
                    finalizeScoringSequence();
                }
                break;
            default:
                scoreState = ScoreState.IDLE;
                break;
        }
    }

    /**
     * Finalizes the scoring sequence by holding the slide in position.
     */
    private void finalizeScoringSequence() {
        slide1.setPower(0.5);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setPower(0.0);
        pivotState = PivotState.IDLE;
        scoreState = ScoreState.IDLE;
        telemetry.addData("Scoring Sequence", "Completed and Slide1 Holding Position");
        telemetry.update();
    }

    // -----------------------
    // Pivot Control
    // -----------------------
    private void handlePivotControl() {
        long now = System.currentTimeMillis();
        switch (pivotState) {
            case MOVING_UP:
                if (now - actionStartTime >= 1300) {
                    pivot.setPower(0.0);
                    pivotState = PivotState.IDLE;
                    telemetry.addData("Pivot", "Moved up and now holding");
                    telemetry.update();
                }
                break;
            case MOVING_DOWN:
                if (now - actionStartTime >= 1000) {
                    pivot.setPower(0.0);
                    pivotState = PivotState.IDLE;
                    telemetry.addData("Pivot", "Moved down and now holding");
                    telemetry.update();
                }
                break;
            case IDLE:
                break;
        }
    }

    /**
     * Handles the pivot button press on Gamepad1 Left Bumper.
     */
    private void handlePivotButtonPress() {
        boolean currentLeftBumperPressed = gamepad1.left_bumper;
        if (currentLeftBumperPressed && pivotState == PivotState.IDLE) {
            movePivotDown();
        }
    }

    /**
     * Initiates moving the pivot up.
     */
    private void startPivotUp() {
        if (pivotState == PivotState.IDLE) {
            pivot.setPower(PIVOT_UP_POWER);
            pivotState = PivotState.MOVING_UP;
            actionStartTime = System.currentTimeMillis();
            telemetry.addData("Pivot", "Started moving up");
            telemetry.update();
        }
    }

    /**
     * Initiates moving the pivot down lightly.
     */
    private void movePivotDown() {
        pivot.setPower(PIVOT_DOWN_POWER);
        pivotState = PivotState.MOVING_DOWN;
        actionStartTime = System.currentTimeMillis();
        telemetry.addData("Pivot", "Started moving down lightly");
        telemetry.update();
    }

    /**
     * Checks if the pivot is in the down position (within a tolerance).
     */
    private boolean isPivotDown() {
        final int PIVOT_DOWN_THRESHOLD = 75;
        return Math.abs(pivot.getCurrentPosition()) <= PIVOT_DOWN_THRESHOLD;
    }

    /**
     * Checks if the pivot is in the up position.
     */
    private boolean isPivotUp() {
        final int PIVOT_UP_THRESHOLD = 100;
        return pivot.getCurrentPosition() > PIVOT_UP_THRESHOLD;
    }

    /**
     * Checks if the pivot is fully down.
     * (Updated: Returns true when pivot encoder is less than 100.)
     */
    private boolean isPivotFullyDown() {
        return pivot.getCurrentPosition() < 100;
    }

    // -----------------------
    // Slide1 Retraction Update
    // -----------------------
    /**
     * When the slide is commanded to retract (target equals SLIDE_INITIAL_POSITION)
     * and it has finished moving (or is within a small tolerance of the starting position),
     * set its mode to RUN_WITHOUT_ENCODER and its zero power behavior to FLOAT.
     * Also clear the retraction flag.
     */
    private void updateSlideRetraction() {
        if (slide1.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                slide1.getTargetPosition() == SLIDE_INITIAL_POSITION &&
                (!slide1.isBusy() || slide1.getCurrentPosition() <= SLIDE_INITIAL_POSITION + SLIDE_RETRACT_TOLERANCE)) {

            slide1.setPower(0);
            slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            isRetracting = false;  // Retraction is complete.
            telemetry.addData("Slide1", "Retracted and now limp");
            telemetry.update();
        }
    }

    // -----------------------
    // Telemetry
    // -----------------------
    private void sendTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Grab Sequence State", grabState);
        telemetry.addData("Score Sequence State", scoreState);
        telemetry.addData("Pivot State", pivotState);
        telemetry.addData("Pivot Position", pivot.getCurrentPosition());
        telemetry.addData("Claw Position", claw.getPosition());
        telemetry.addData("Arm Position", arm.getPosition());
        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.addData("Rotate Position", rotate.getPosition());
        telemetry.addData("Slide1 Mode", slide1.getMode());
        telemetry.addData("Slide1 Target", slide1.getTargetPosition());
        telemetry.addData("Slide1 Current", slide1.getCurrentPosition());
        telemetry.addData("Slide1 isBusy", slide1.isBusy());
        telemetry.addData("Rotate Manual Position", rotateManualPosition);
        telemetry.addData("Is Retracting", isRetracting);
        telemetry.update();
    }
}