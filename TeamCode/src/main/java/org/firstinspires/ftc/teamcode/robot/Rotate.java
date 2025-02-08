package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Rotate {
    private final Servo claw;
    private Position position;

    public Rotate(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("rotate");
        claw.setDirection(Servo.Direction.FORWARD);
        position = Position.YES;
        setPosition(position);
    }

    public enum Position {
        YES(0.78);

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
