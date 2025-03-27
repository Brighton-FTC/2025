package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Setup Specimen Auto Arm Position")
public class SpecimenArmSetup extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "linear_slide_motor");
        GrabberComponent grabber = new GrabberComponent(hardwareMap, "left_claw_servo", "right_claw_servo", "rotator_servo");

        linearSlide.upSpecimen();

        while (!linearSlide.atSetPoint() && !isStopRequested()) {
            linearSlide.run();
        }

        grabber.tilt();
        sleep(300);
        linearSlide.down();

        while (!linearSlide.atSetPoint() && !isStopRequested()) {
            linearSlide.run();
        }
    }
}
