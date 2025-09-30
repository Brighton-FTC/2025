package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


//@Config allows real time optimisation towards variables,
@Config
public class GrabberComponent {
    
    public static double CLAW_TURN_AMOUNT = 0.5;

    //Initialising the hardware claw,
    private final Servo claw; 

    //Create constructor for access in other classes or OpModes. (This is the base setting for the servo)
    public GrabberComponent(HardwareMap hardwareMap, String servoId) {
        claw = hardwareMap.servo.get(servoId);
        claw.setDirection(Servo.Direction.REVERSE);
    }

    //Construct the method that acts on the servo. We call this by doing something.grab(), where something is initialised to 
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
