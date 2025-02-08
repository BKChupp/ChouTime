package org.firstinspires.ftc.teamcode.actions;


import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.*;


@Autonomous
public class ActionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Rotate rotate = new Rotate(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);

        Actions.runBlocking(
                new SequentialAction(
                        new ClawAction(claw, Claw.Position.CLOSED),
                        new RotateAction(rotate, Rotate.Position.YES),
                        new ArmAction(arm, Arm.Position.SCORE),
                        new WristAction(wrist, Wrist.Position.SCORE),
                        new SlideAction(slides, 1000),
                        new DelayAction(5000)
                )
        );
    }
}
