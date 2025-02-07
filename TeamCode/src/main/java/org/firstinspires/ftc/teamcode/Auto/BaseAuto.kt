package org.firstinspires.ftc.teamcode.Auto

import com.acmerobotics.roadrunner.*
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Actions.*
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.Robot.Claw
import org.firstinspires.ftc.teamcode.Robot.Pivot
import org.firstinspires.ftc.teamcode.Robot.Slides
import org.firstinspires.ftc.teamcode.Robot.Wrist

interface PositionData {
    val startingPosition: Pose2d
    val scoringPosition: Vector2d
    val grabSpecimenLocation: Vector2d
    val scoreSpec: Double
    val HumanP: Double
    val firstSpecimenHeading: Double
    val secondSpecimenHeading: Double
    val thirdSpecimenHeading: Double
    val firstSpecReachDistance: Int
    val secondSpecReachDistance: Int
    val thirdSpecReachDistance: Int
}

data class RedPositionData(
    override val startingPosition: Pose2d = Pose2d(0.0, -62.75, Math.toRadians(90.0)),
    override val scoringPosition: Vector2d = Vector2d(0.0, -55.0),
    override val grabSpecimenLocation: Vector2d = Vector2d(36.0, -50.0),
    override val scoreSpec: Double = Math.toRadians(90.0),
    override val HumanP: Double = Math.toRadians(-45.0),
    override val firstSpecimenHeading: Double = Math.toRadians(0.0),
    override val secondSpecimenHeading: Double = Math.toRadians(85.0),
    override val thirdSpecimenHeading: Double = Math.toRadians(115.0),
    override val firstSpecReachDistance: Int = 500, // -1725,
    override val secondSpecReachDistance: Int = 1000, // -1660,
    override val thirdSpecReachDistance: Int = 1200, //-1800,
) : PositionData

data class BluePositionData(
    override val startingPosition: Pose2d = Pose2d(0.0, -62.75, Math.toRadians(90.0)),
    override val scoringPosition: Vector2d = Vector2d(0.0, -45.0),
    override val grabSpecimenLocation: Vector2d = Vector2d(36.0, -45.0),
    override val scoreSpec: Double = Math.toRadians(90.0),
    override val HumanP: Double = Math.toRadians(-45.0),
    override val firstSpecimenHeading: Double = Math.toRadians(135.0),
    override val secondSpecimenHeading: Double = Math.toRadians(85.0),
    override val thirdSpecimenHeading: Double = Math.toRadians(115.0),
    override val firstSpecReachDistance: Int = 500, // -1725,
    override val secondSpecReachDistance: Int = 1000, // -1660,
    override val thirdSpecReachDistance: Int = 1200, //-1800,
) : PositionData

open class BaseAuto(private val positionData: PositionData) : LinearOpMode() {


    override fun runOpMode() {
        val pivot = Pivot(hardwareMap)
        val pivotUpAction = PivotUpAction(hardwareMap)
        val pivotDownAction = PivotDownAction(hardwareMap)

        waitForStart()

        val claw = Claw(hardwareMap)
        val slide = Slides(hardwareMap)
        val wrist = Wrist(hardwareMap)

        var initialPose = positionData.startingPosition
        val drive = MecanumDrive(hardwareMap, initialPose)

//        slide.setPosition(2200)
//
//       sleep(100)

        val toScoringPositionAction = drive
            .actionBuilder(initialPose)
            .strafeToSplineHeading(
                positionData.scoringPosition,
                positionData.scoreSpec
            )
            .build()

        var performMovement = drive
            .actionBuilder(initialPose)
            .stopAndAdd(
                SequentialAction(
                    ParallelAction(
                        ClawAction(claw, Claw.Position.CLOSED),
                        WristAction(wrist, Wrist.Position.SCORE),
                        PivotUpAction(hardwareMap),
                        toScoringPositionAction,
                    ),
                )
            )

        runBlocking(performMovement.build())

        val toSpecPositionAction = drive
            .actionBuilder(Pose2d(0.0, -55.0, Math.toRadians(90.0)))
            .strafeTo(positionData.grabSpecimenLocation)
            .build()

        performMovement = drive
            .actionBuilder(initialPose)
            .stopAndAdd(
                SequentialAction(
                    ParallelAction(
                        ClawAction(claw, Claw.Position.CLOSED),
                        WristAction(wrist, Wrist.Position.SCORE),
                        PivotDownAction(hardwareMap),
                        toSpecPositionAction,
                    ),
                )
            )

        runBlocking(performMovement.build())

        val SpecsToHuman = drive
            .actionBuilder(Pose2d(36.0, -55.0, Math.toRadians(90.0)))
            .turnTo(positionData.firstSpecimenHeading)
            .build()

        runBlocking(SpecsToHuman)

        /*
        val toPushSpecsPositionAction = drive
            .actionBuilder(drive.poseHistory.last)
            .strafeToConstantHeading(positionData.grabSpecs)
            .build()

        performMovement = drive
            .actionBuilder(drive.poseHistory.last)
            .stopAndAdd(
                SequentialAction(
                    ParallelAction(
//                        ClawAction(claw, Claw.Position.CLOSED),
//                        WristAction(wrist, Wrist.Position.SCORE),
                        PivotDownAction(hardwareMap),
                        toPushSpecsPositionAction,
                    ),
                )
            )

        runBlocking(performMovement.build())
         */
    }
}
