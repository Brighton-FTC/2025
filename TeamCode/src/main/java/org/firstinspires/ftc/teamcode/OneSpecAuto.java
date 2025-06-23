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
    double subY = -35;
    double loop_x = 36;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, startY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "vertical_slide_motor", "vertical_slide_sensor");


        GrabberComponent grabber = new GrabberComponent(hardwareMap, "claw_servo");

        Action startToSub = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .afterDisp(0, linearSlide::up)
                .splineToConstantHeading(new Vector2d(Roriginal_x, subY), startHeading)
                .build();

        Action startToPark = drive.actionBuilder(new Pose2d(Roriginal_x, subY, startHeading))
                .afterDisp(0, linearSlide::down)
                .afterDisp(0, grabber::toggleClaw)
                .splineToConstantHeading(new Vector2d(loop_x+10,startY), startHeading)
                .build();


        grabber.toggleClaw();

        waitForStart();

        new Thread(() -> {
            while (!isStopRequested()) {
                linearSlide.run();
                sleep(20);
            }
        }).start();

        Actions.runBlocking(startToSub);
        Actions.runBlocking(new InstantAction(linearSlide::score));
        Actions.runBlocking(new SleepAction(0.3));
        Actions.runBlocking(new InstantAction(grabber::toggleClaw));
        Actions.runBlocking(startToPark);
//        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
