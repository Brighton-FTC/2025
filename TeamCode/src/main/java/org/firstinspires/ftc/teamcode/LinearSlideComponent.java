package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class LinearSlideComponent {

    public static double kp = 0.0015;

    public static double ki = 0;

    public static double kd = 0;

    public static double UP_POSITION = 2000;

    public static double DOWN_POSITION = 0;

    public static double SCORE_POSITION = 1500;

    private PIDController controller = new PIDController(0, 0, 0);

    private Motor slideMotor;

    private TouchSensor touchSensor;

    private boolean isrunningPID = true;

    private double input = 0;

    public LinearSlideComponent(HardwareMap hardwareMap, String motorid, String touchSensorid) {
        slideMotor = new Motor(hardwareMap, motorid);
        touchSensor = hardwareMap.touchSensor.get(touchSensorid);
        slideMotor.resetEncoder();
    }

    public void run() {
        if (isrunningPID) {
            controller.setPID(kp, ki, kd);

            double power = controller.calculate(slideMotor.getCurrentPosition());

            if (touchSensor.isPressed()) {
                slideMotor.set(Math.max(power, 0));
                slideMotor.stopAndResetEncoder();
            } else {
                slideMotor.set(power);
            }
        } else {
            if (touchSensor.isPressed()) {
                slideMotor.set(0);
            } else {
                slideMotor.set(-input);
                input = 0;
            }
        }
    }

    public void up() {
        isrunningPID = true;
        controller.setSetPoint(UP_POSITION);
    }

    public void down() {
        isrunningPID = true;
        controller.setSetPoint(DOWN_POSITION);
    }

    public void score() {
        isrunningPID = true;
        controller.setSetPoint(SCORE_POSITION);
    }
    public void rawInput(double input) {
        isrunningPID = false;
        this.input = input;
    }

    public Motor getMotor() {
        return slideMotor;
    }

    public double getSetPoint() {
        return controller.getSetPoint();
    }

    public boolean atSetPoint() {
        return controller.atSetPoint();
    }

    public boolean isTouchSensorPressed() {
        return touchSensor.isPressed();
    }
}

