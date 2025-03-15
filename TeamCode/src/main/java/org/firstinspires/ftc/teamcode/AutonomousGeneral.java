package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;


@Config
@Autonomous
public class AutonomousGeneral extends LinearOpMode {
    protected MecanumDrive drive;

    public static double basketX = 3; // suboptimal solution to drift
    public static double basketY = 26;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);

        drive = new MecanumDrive(hardwareMap, initialPose);

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "linear_slide_motor");
        GrabberComponent grabber = new GrabberComponent(hardwareMap, "left_claw_servo", "right_claw_servo", "rotator_servo");

        Action autonomousAction = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(12, basketY / 2), Math.PI / 2)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(basketX, basketY), Math.PI)
                .build();

        waitForStart();

        Actions.runBlocking(autonomousAction);

        linearSlide.up();

        while (!linearSlide.atSetPoint() && !isStopRequested()) {
            linearSlide.run();
            sleep(20);
        }

        grabber.down(); // for some mystical reason the grabber won't rotate forwards unless this has been called first
        sleep(20);
        grabber.forward();
        sleep(3000);

        grabber.reset();
        sleep(1000);

        grabber.down();

        linearSlide.down();

        while (!linearSlide.atSetPoint() && !isStopRequested()) {
            linearSlide.run();
            sleep(20);
        }
    }
}
