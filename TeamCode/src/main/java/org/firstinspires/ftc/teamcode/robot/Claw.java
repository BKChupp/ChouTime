package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo claw;
    private Position position;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
        position = Position.CLOSED;
        setPosition(position);
    }

    public enum Position {
        OPEN(0.5),
        CLOSED(0.27);

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
        claw.setPosition(position.getPosition());
    }

    public double getRawPosition() {
        return claw.getPosition();
    }

    public void setRawPosition(double position) {
        claw.setPosition(position);
    }
}
