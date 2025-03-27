package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class GrabberComponent {
    public static double LEFT_CLAW_TURN_AMOUNT = 1;
    public static double RIGHT_CLAW_TURN_AMOUNT = 0.4;

    public static double ROTATOR_FORWARD_TURN_AMOUNT = 0.9;
    public static double ROTATOR_SPECIMEN_SCORE_TURN_AMOUNT = 1.0;
    public static double ROTATOR_DOWN_TILTED_TURN_AMOUNT = 0.2;

    private final Servo leftClaw, rightClaw, rotator;


    public GrabberComponent(HardwareMap hardwareMap, String leftId, String rightId, String rotatorId) {
        leftClaw = hardwareMap.servo.get(leftId);
        rightClaw = hardwareMap.servo.get(rightId);
        rotator = hardwareMap.servo.get(rotatorId);


        leftClaw.setDirection(Servo.Direction.REVERSE);
    }

    public void grab() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);

    }

    public void reset() {
        leftClaw.setPosition(LEFT_CLAW_TURN_AMOUNT);
        rightClaw.setPosition(RIGHT_CLAW_TURN_AMOUNT);

    }

    public void toggleClaw() {
        if (leftClaw.getPosition() == 0) {
            reset();
        } else {
            grab();
        }
    }

    public void down() {
        rotator.setPosition(0);
    }

    public void forward() {
        rotator.setPosition(ROTATOR_FORWARD_TURN_AMOUNT);
    }

    public void scoreSpecimen() {
        rotator.setPosition(ROTATOR_SPECIMEN_SCORE_TURN_AMOUNT);
    }

    public void tilt() {
        rotator.setPosition(ROTATOR_DOWN_TILTED_TURN_AMOUNT);
    }

    public void toggleRotatorForward() {
        if (rotator.getPosition() == 0) {
            forward();
        } else {
            down();
        }
    }

    public void toggleRotatorUp() {
        if (isDown()) {
            scoreSpecimen();
        } else {
            down();
        }
    }

    public boolean isClosed() {
        return leftClaw.getPosition() == 0;
    }

    public boolean isDown() {
        return rotator.getPosition() == 0;
    }

    public Servo getLeftServo() {
        return leftClaw;
    }

    public Servo getRightServo() {
        return rightClaw;
    }

    public Servo getRotatorServo() {
        return rotator;
    }
}
