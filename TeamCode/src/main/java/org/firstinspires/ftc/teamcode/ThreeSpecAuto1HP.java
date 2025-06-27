package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "3 Specimen Autonomous + 1 HP", preselectTeleOp = "Teleop")
public class ThreeSpecAuto1HP extends LinearOpMode {
    public static double startY = -72;
    double startHeading = Math.toRadians(90);

    double downTangent = Math.toRadians(270);

    double Roriginal_x = 10;
    double subY = -40;
    double loop_x = 36;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, startY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "vertical_slide_motor", "vertical_slide_sensor");

        linearSlide.resetSlideEncoder();

        GrabberComponent grabber = new GrabberComponent(hardwareMap, "claw_servo");

        Action scoreSpec = new SequentialAction(new InstantAction(linearSlide::score), new SleepAction(0.15), new InstantAction(grabber::reset));

        Action startToSub = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .afterTime(0, linearSlide::up)
                .splineToConstantHeading(new Vector2d(Roriginal_x, subY-1), startHeading)
                .afterTime(0, linearSlide::score)
                .build();

        Action cycle1 = drive.actionBuilder(new Pose2d(Roriginal_x, subY-1, startHeading))
                .setTangent(Math.toRadians(315))
                .afterTime(0, linearSlide::down)
                .waitSeconds(0.3)
                .splineToConstantHeading(new Vector2d(loop_x+10, -48), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 10, -19), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+13, -19), startHeading)
                .setTangent(Math.toRadians(320))
                .splineToConstantHeading(new Vector2d(loop_x + 24, -60), startHeading)
                .splineToSplineHeading(new Pose2d(loop_x+5, startY-22, downTangent), startHeading)
                .build();



        Action specCycle1 = drive.actionBuilder(new Pose2d(Roriginal_x-2, subY-2, startHeading))
                .afterTime(0, linearSlide::down)
                .waitSeconds(0.3)
                .setTangent(downTangent)
                .splineToLinearHeading(new Pose2d(35, startY-21.7, downTangent), downTangent)
                .afterTime(0, grabber::grab)
                .waitSeconds(0.5)
                .afterTime(0, linearSlide::up2)
                .waitSeconds(0.3)
                .splineToSplineHeading(new Pose2d(Roriginal_x-1, subY-10, startHeading), 270)
                .splineToConstantHeading(new Vector2d(Roriginal_x-5, subY-1), startHeading)
                .build();


        Action specCycle2 = drive.actionBuilder(new Pose2d(loop_x+5, startY, downTangent))
                .afterTime(0, linearSlide::up)
                .splineToSplineHeading(new Pose2d(Roriginal_x-2, subY-10, startHeading), 270)
                .splineToConstantHeading(new Vector2d(Roriginal_x-2, subY-1), startHeading)
                .afterTime(0, linearSlide::score)
                .build();



        Action scoreToPark = drive.actionBuilder(new Pose2d(Roriginal_x-4, subY-2, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x+5,startY+1), startHeading)
                .build();


        grabber.toggleClaw();

        waitForStart();

        new Thread(() -> {
            while (!isStopRequested()) {
                linearSlide.run();
                sleep(20);
            }
        }).start();

        Actions.runBlocking(new SequentialAction(startToSub, scoreSpec, cycle1, new InstantAction(grabber::grab), new SleepAction(0.5), specCycle2, new InstantAction(grabber::reset), specCycle1, new InstantAction(linearSlide::score), new SleepAction(0.2), new InstantAction(grabber::reset), new InstantAction(linearSlide::down), scoreToPark));

//        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
