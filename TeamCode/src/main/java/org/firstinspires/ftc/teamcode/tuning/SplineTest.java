package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-35.0, -62.75, 90);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            telemetry.addData("drive class", TuningOpModes.DRIVE_CLASS);
            telemetry.update();

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToSplineHeading(new Vector2d(-57.5, -53.5), Math.toRadians(45.0))
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);
            telemetry.addData("drive class", TuningOpModes.DRIVE_CLASS);
            telemetry.update();

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToX(10.0)
                            .build());
        } else {
            throw new RuntimeException();
        }

        telemetry.addData("drive class", TuningOpModes.DRIVE_CLASS);
        telemetry.update();

    }
}
