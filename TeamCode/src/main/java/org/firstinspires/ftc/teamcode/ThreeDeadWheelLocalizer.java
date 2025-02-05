package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        // These offsets represent the positions of your dead wheels in encoder ticks.
        // Adjust them based on your robot’s geometry.
        public double par0YTicks = -42384.86617670765;  // y-pos of parallel encoder #1 in ticks
        public double par1YTicks = 39967.63521143594; // y-pos of parallel encoder #2 in ticks
        public double perpXTicks = -45029.05247759617; // x-pos of perpendicular encoder in ticks
    }

    public static Params PARAMS = new Params();

    // Encoders for the 3 dead wheels
    public final Encoder par0, par1, perp;

    public final double inPerTick;

    // Most recent known robot pose
    private Pose2d pose;

    // Internal tracking from last cycle
    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;

    /**
     * Construct a ThreeDeadWheelLocalizer with a given starting pose.
     *
     * @param hardwareMap hardware map to fetch encoders
     * @param inPerTick   conversion factor from encoder ticks to inches
     * @param startPose   initial pose estimate of the robot
     */
    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d startPose) {
        // Retrieve motors from config to get their embedded encoders
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "bl")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "br")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "fl")));

        // Reverse direction on encoders if necessary
        // For example, if your perpendicular wheel is physically oriented in a way
        // that needs reversing:
        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        // Set our local pose to the passed-in initial pose
        this.pose = startPose;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Updates the localizer and returns the robot’s velocity (dx/dt, dy/dt, dHeading/dt).
     *
     * @return the current velocity as a {@link PoseVelocity2d}
     */
    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        // Record raw inputs for logging/debugging
        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS",
                new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        // On first run, just initialize encoders
        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            // Return zero velocity until next update
            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        // Calculate change in encoder ticks since last cycle
        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        // Update last known positions
        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        // The math that calculates the delta in x/y based on your wheel arrangement
        // par0YTicks, par1YTicks = Y offsets for parallel wheels
        // perpXTicks = X offset for perpendicular wheel
        // ...
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        // X and Y displacements (and velocities in element [1]) for your 2D position
                        new DualNum<Time>(new double[]{
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta)
                                        / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity)
                                        / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[]{
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks)
                                        * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks)
                                        * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                // Heading change (and heading velocity) from difference between par0 & par1
                new DualNum<>(new double[]{
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        // Update our stored pose by applying the computed twist
        pose = pose.plus(twist.value());

        // Return current velocity
        return twist.velocity().value();
    }
}
