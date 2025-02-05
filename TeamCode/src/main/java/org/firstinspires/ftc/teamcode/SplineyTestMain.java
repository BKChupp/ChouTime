package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ActionHardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class SplineyTestMain extends OpMode {
    private ActionHardware pivot;

    @Override
    public void init() {
        Pose2d beginPose = new Pose2d(0.0, -62.75, Math.toRadians(90.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
    }

    @Override
    public void loop() {
        Pose2d beginPose = new Pose2d(0.0, -62.75, Math.toRadians(90.0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        final Action ScorePreload = drive
                .actionBuilder(beginPose)
//                .stopAndAdd()
                .lineToY(-40)
                .build();


        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // score specimen
                        .lineToY(-40)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(37.0, -40))
                        .waitSeconds(.1)
                        .lineToY(-12)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(47, -12))
                        .waitSeconds(.1)
                        .lineToY(-55)
                        .waitSeconds(.1)
                        .lineToY(-12)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(57, -12))
                        .waitSeconds(.1)
                        .lineToY(-55)
                        .waitSeconds(.1)
                        .lineToY(-12)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(66, -12))
                        .waitSeconds(.1)
                        .lineToY(-55)
                        .waitSeconds(.1)
                        // wall position
                        .strafeTo(new Vector2d(37, -55))
                        .waitSeconds(.1)
                        .lineToY(-58)
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
                        // release
                        .build());

        telemetry.addData("drive class", TuningOpModes.DRIVE_CLASS);
        telemetry.update();

    }
}