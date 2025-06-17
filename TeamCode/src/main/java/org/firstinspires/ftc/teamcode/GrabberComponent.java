package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class GrabberComponent {
    public static double CLAW_TURN_AMOUNT = 0.5;


    private final Servo claw;


    public GrabberComponent(HardwareMap hardwareMap, String servoId) {
        claw = hardwareMap.servo.get(servoId);
        claw.setDirection(Servo.Direction.REVERSE);
    }

    public void grab() {
        claw.setPosition(0);

    }

    public void reset() {
        claw.setPosition(CLAW_TURN_AMOUNT);

    }

    public void toggleClaw() {
        if (claw.getPosition() == 0) {
            reset();
        } else {
            grab();
        }
    }



    public boolean isClosed() {
        return claw.getPosition() == 0;
    }

    public Servo getServo() {
        return claw;
    }

}
