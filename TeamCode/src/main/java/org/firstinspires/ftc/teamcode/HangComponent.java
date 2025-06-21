package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HangComponent {
    public static int STOP_POSITION = 1000; // TODO: tune

    private Motor hangMotor;
    private Servo hangServo;

    public HangComponent(HardwareMap hardwareMap, String hangMotorId, String hangServoId) {
        hangMotor = new Motor(hardwareMap, "hang_motor");
        hangMotor.resetEncoder();
        hangMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        hangServo = hardwareMap.servo.get("hang_servo");
    }

    public void release() {
        hangServo.setPosition(1);
    }

    public void winch() {
        if (hangMotor.getCurrentPosition() < STOP_POSITION) {
            hangMotor.set(1);
        } else {
            hangMotor.set(0);
        }
    }

    public boolean isReleased() {
        return hangServo.getPosition() != 0;
    }

    public boolean atSetPoint() {
        return hangMotor.getCurrentPosition() >= STOP_POSITION;
    }
}
