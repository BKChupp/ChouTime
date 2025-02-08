package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.actions.*;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.lang.Math;

@Autonomous
public class SplineyTestMain extends LinearOpMode {
    private ActionHardware pivot;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0.0, -62.75, Math.toRadians(90.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Rotate rotate = new Rotate(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(
//                                new SequentialAction()
                                new ParallelAction(
                                        new ClawAction(claw, Claw.Position.CLOSED),
                                        new RotateAction(rotate, Rotate.Position.YES),
                                        new ArmAction(arm, Arm.Position.SCORE),
                                        new WristAction(wrist, Wrist.Position.SCORE),
                                        new SlideAction(slides, 1490)
                                )
                        )
                        // score specimen
                        .lineToY(-55)
                        .waitSeconds(.1)
                        .lineToY(-50)
                        // release the specimen
                        .stopAndAdd(
                                new ParallelAction(
                                        new ClawAction(claw, Claw.Position.OPEN),
                                        new ArmAction(arm, Arm.Position.RELEASE),
                                        new SlideAction(slides, 0)
                                )
                        )
                        .waitSeconds(1)
                        // backup the bot
                        .lineToY(-40)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(37.0, -40))
                        .waitSeconds(.1)
                        .lineToY(-12)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(47, -12))
                        .waitSeconds(.1)
                        // push first sample backwards
                        .lineToY(-55)
                        .waitSeconds(.1)
                        .lineToY(-12)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(57, -12))
                        .waitSeconds(.1)
                        // push second sample backwards
                        .lineToY(-55)
                        .waitSeconds(.1)
                        .lineToY(-12)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(66, -12))
                        .waitSeconds(.1)
                        // push third sample backwards
                        .lineToY(-55)
                        .waitSeconds(.1)
                        // wall position
                        .strafeTo(new Vector2d(37, -55))
                        .waitSeconds(.1)
                        .lineToY(-58)
                        .stopAndAdd(
                                new ParallelAction(
                                        new RotateAction(rotate, Rotate.Position.YES),
                                        new ArmAction(arm, Arm.Position.LINE_UP),
                                        new WristAction(wrist, Wrist.Position.LINE_UP)
                                )
                        )
                        //grab
                        .waitSeconds(.2)
                        .strafeTo(new Vector2d(5, -40))
                        // scoring position
                        .waitSeconds(.1)
                        // release
                        // wall position
                        .strafeTo(new Vector2d(37, -55))
                        .waitSeconds(.1)
                        .lineToY(-58)
                        //grab
                        .waitSeconds(.2)
                        .strafeTo(new Vector2d(7, -40))
                        // scoring position
                        .waitSeconds(.1)
                        // release
                        // wall position
                        .strafeTo(new Vector2d(37, -55))
                        .waitSeconds(.1)
                        .lineToY(-58)
                        // grab
                        .waitSeconds(.2)
                        .strafeTo(new Vector2d(9, -40))
                        // scoring position
                        .waitSeconds(.1)
                        // release
                        // wall position
                        .strafeTo(new Vector2d(37, -55))
                        .waitSeconds(.1)
                        .lineToY(-59)
                        // grab
                        .waitSeconds(.2)
                        .strafeTo(new Vector2d(11, -40))
                        // scoring position
                        .waitSeconds(.1)
                        .stopAndAdd(new SequentialAction(new PivotDownAction(hardwareMap)))
                        // release
                        .build());

        telemetry.addData("drive class", TuningOpModes.DRIVE_CLASS);
        telemetry.update();

    }
}
