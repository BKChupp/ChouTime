//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
//
//public final class Auto extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d beginPose = new Pose2d(0, 0, 0);
//        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
//            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//
//            waitForStart();
//
//            Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .lineToX(10.0)
//                        .build());
//
//        }
//    }
//}
