package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private final Servo wrist;
    private Position position;

    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
        position = Position.INTAKE;
        setPosition(position);
    }

    public enum Position {
        INTAKE(0.9),
        HOVER(0.94),
        GRAB_WALL(0.85),
        SCORE(0.94),
        LINE_UP(0.75);

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
        wrist.setPosition(position.getPosition());
    }

    public void setRawPosition(double position) {
        wrist.setPosition(position);
    }

    public double getRawPosition() {
        return wrist.getPosition();
    }

    public Position getPosition() {
        return position;
    }
}
