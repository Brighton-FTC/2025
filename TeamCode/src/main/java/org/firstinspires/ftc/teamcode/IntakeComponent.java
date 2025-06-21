package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeComponent {
    private Motor intakeMotor;
    private State state = State.STOPPED;

    public IntakeComponent(HardwareMap hardwareMap, String intakeMotorId) {
        intakeMotor = new Motor(hardwareMap, intakeMotorId);
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
}
