package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class LinearSlideComponent {

    public static double kp = 0;

    public static double ki = 0;

    public static double kd = 0;

    private double setpoint = 0;

    public static double UP_POSITION = 0;

    public static double DOWN_POSITION = 0;

    private PIDController controller = new PIDController(0, 0, 0);

    private Motor slideMotor;

    private TouchSensor touchSensor;

    private boolean isrunningPID;

    public LinearSlideComponent(HardwareMap hardwareMap, String motorid, String touchSensorid) {
        slideMotor = new Motor(hardwareMap, motorid);
        touchSensor = hardwareMap.touchSensor.get(touchSensorid);

    }

    public void run() {
        if (isrunningPID) {
            controller.setPID(kp, ki, kd);
            slideMotor.set(controller.calculate(slideMotor.getCurrentPosition()));
        } else if (!touchSensor.isPressed()) {
            slideMotor.set(-0.5);

        } else {
            slideMotor.set(0);
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

    public void downToSwitch() {
        isrunningPID = false;
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
}

