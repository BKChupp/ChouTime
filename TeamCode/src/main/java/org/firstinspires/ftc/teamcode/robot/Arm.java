package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final Servo arm;
    private Position position;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.servo.get("arm");
        arm.setDirection(Servo.Direction.FORWARD);
        position = Position.INTAKE;
        setPosition(position);
    }

    public enum Position {
        INTAKE(0.35),
        HOVER(0.43),
        GRAB_WALL(0.95),
        SCORE(0.45),
        RELEASE(0.50),
        LINE_UP(0.98);

        private final double position;

        Position(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public void setPosition(Position position) {
        this.position = position;
        arm.setPosition(position.getPosition());
    }

    public double getRawPosition() {
        return arm.getPosition();
    }

    public void setRawPosition(double position) {
        arm.setPosition(position);
    }
}
