package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class GrabberComponent {
    public static double CLAW_TURN_AMOUNT = -1;


    private final Servo Claw;


    public GrabberComponent(HardwareMap hardwareMap, String servoId) {
        Claw = hardwareMap.servo.get(servoId);

    }

    public void grab() {
        Claw.setPosition(0);

    }

    public void reset() {
        Claw.setPosition(CLAW_TURN_AMOUNT);

    }

    public void toggleClaw() {
        if (Claw.getPosition() == 0) {
            reset();
        } else {
            grab();
        }
    }



    public boolean isClosed() {
        return Claw.getPosition() == 0;
    }

    public Servo getServo() {
        return Claw;
    }

}
