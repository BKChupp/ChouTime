package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.robot.Arm;

public class ArmAction implements Action {
    private final Arm arm;
    private final Arm.Position armPosition;

    public ArmAction(Arm arm, Arm.Position armPosition) {
        this.arm = arm;
        this.armPosition = armPosition;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        arm.setPosition(armPosition);
        return false;
    }
}
