package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "1 Specimen Autonomous", preselectTeleOp = "Teleop")
public class OneSpecAuto extends LinearOpMode {
    public static double startY = -72;
    double startHeading = Math.toRadians(90);

    double Roriginal_x = 10;
    double subY = -41;
    double loop_x = 36;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, startY-2, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "vertical_slide_motor", "vertical_slide_sensor");


        GrabberComponent grabber = new GrabberComponent(hardwareMap, "claw_servo");

        linearSlide.resetSlideEncoder();

        Action startToSub = drive.actionBuilder(new Pose2d(Roriginal_x, startY-2, startHeading))
                .afterDisp(0, linearSlide::up)
                .splineToConstantHeading(new Vector2d(Roriginal_x, subY-1), startHeading)
                .build();

        Action startToPark = drive.actionBuilder(new Pose2d(Roriginal_x, subY-1, startHeading))
                .afterDisp(0, linearSlide::down)
                .afterDisp(0, grabber::toggleClaw)
                .splineToConstantHeading(new Vector2d(loop_x+10,startY), startHeading)
                .build();


        grabber.grab();

        waitForStart();

        new Thread(() -> {
            while (!isStopRequested()) {
                linearSlide.run();
                sleep(20);
            }
        }).start();

        Actions.runBlocking(startToSub);
        Actions.runBlocking(new InstantAction(linearSlide::score));
        Actions.runBlocking(new SleepAction(1));
        Actions.runBlocking(new InstantAction(grabber::grab));
        Actions.runBlocking(startToPark);
//        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
