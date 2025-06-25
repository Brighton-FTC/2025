package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeComponent {
    private Motor intakeMotor;

    private final Servo intakeServo;
    private State state = State.STOPPED;


    public IntakeComponent(HardwareMap hardwareMap, String intakeMotorId, String intakeServoId) {
        intakeMotor = new Motor(hardwareMap, intakeMotorId);

        intakeServo = hardwareMap.servo.get(intakeServoId);
    }

    public void run() {
        intakeMotor.set(state.power);
    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    public enum State {
        FORWARD(1),
        STOPPED(0),
        REVERSE(-1);

        public final double power;

        State(double power) {
            this.power = power;
        }
    }

    public void turnServo (double turnAmount){intakeServo.setPosition(turnAmount);}
}
