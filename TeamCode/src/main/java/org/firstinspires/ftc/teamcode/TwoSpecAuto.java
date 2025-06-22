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
@Autonomous(name = "2 Specimen Autonomous", preselectTeleOp = "Teleop")
public class TwoSpecAuto extends LinearOpMode {
    public static double startY = -72;
    double startHeading = Math.toRadians(90);

    double downTangent = Math.toRadians(270);

    double Roriginal_x = 10;
    double subY = -38;
    double loop_x = 36;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, startY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "vertical_slide_motor", "vertical_slide_sensor");


        GrabberComponent grabber = new GrabberComponent(hardwareMap, "claw_servo");

        Action startToSub = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .afterDisp(10, linearSlide::up)
                .splineToConstantHeading(new Vector2d(Roriginal_x, subY), startHeading)
                .build();

        Action subToSpike = drive.actionBuilder(new Pose2d(Roriginal_x, subY, startHeading))
                .splineToSplineHeading(new Pose2d(Roriginal_x, -48, startHeading), downTangent)
                .splineToConstantHeading(new Vector2d(36, -48), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x, -10), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+5, startY), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+10, -10), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+15, startY), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+20, -10), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+22, startY), startHeading)
                .splineToSplineHeading(new Pose2d(loop_x+5,startY, downTangent), 0)
                .build();

        Action specCycle1 = drive.actionBuilder(new Pose2d(loop_x+5, startY, startHeading))
                .afterDisp(10, linearSlide::up)
                .splineToSplineHeading(new Pose2d(Roriginal_x, -34, startHeading), 0)
                .build();

        Action specCycle2 = drive.actionBuilder(new Pose2d(Roriginal_x, -34, startHeading))
                .splineToSplineHeading(new Pose2d(loop_x+5,startY, downTangent), 0)
                .afterDisp(10, linearSlide::up)
                .splineToSplineHeading(new Pose2d(Roriginal_x, -34, startHeading), 0)
                .build();

        Action startToPark = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x+5,startY), startHeading)
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
        Actions.runBlocking(new SleepAction(0.5));
        Actions.runBlocking(new InstantAction(grabber::toggleClaw));
        Actions.runBlocking(new InstantAction(linearSlide::down));
        Actions.runBlocking(subToSpike);
        Actions.runBlocking(new InstantAction(grabber::toggleClaw));

        Actions.runBlocking(specCycle1);
        Actions.runBlocking(new InstantAction(linearSlide::score));
        Actions.runBlocking(new SleepAction(0.5));
        Actions.runBlocking(new InstantAction(grabber::toggleClaw));
        Actions.runBlocking(new InstantAction(linearSlide::down));
        Actions.runBlocking(startToPark);
//        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
