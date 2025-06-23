package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
    double subY = -35;
    double loop_x = 36;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, startY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "vertical_slide_motor", "vertical_slide_sensor");


        GrabberComponent grabber = new GrabberComponent(hardwareMap, "claw_servo");

        Action startToSub = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .afterDisp(10, linearSlide::up)
                .splineToConstantHeading(new Vector2d(Roriginal_x, subY), startHeading)
                .afterDisp(0, linearSlide::score)
                .build();

        Action cycle1 = drive.actionBuilder(new Pose2d(Roriginal_x, subY, startHeading))
                .afterDisp(0, linearSlide::down)
                .splineToConstantHeading(new Vector2d(Roriginal_x, -60), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 11, -19), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 13, -60), startHeading)
                .build();

// Second spike
        Action cycle2 = drive.actionBuilder(new Pose2d(loop_x + 13, -60, startHeading))

                .splineToConstantHeading(new Vector2d(loop_x + 13, -19), startHeading)

                .splineToConstantHeading(new Vector2d(loop_x + 23, -60), startHeading)
                .build();

// Third spike
        Action cycle3 = drive.actionBuilder(new Pose2d(loop_x + 23, -60, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x + 24, -19), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 33, -60), startHeading)
                .splineToSplineHeading(new Pose2d(loop_x+5, startY-20, downTangent), 0)
                .build();

        Action specCycle1 = drive.actionBuilder(new Pose2d(loop_x+5, startY, downTangent))
                .afterDisp(0, linearSlide::up)
                .splineToSplineHeading(new Pose2d(Roriginal_x+2, -34, startHeading), 270)
                .afterDisp(0, linearSlide::score)
                .build();

        Action specCycleN = drive.actionBuilder(new Pose2d(Roriginal_x+2, -34, startHeading))
                .afterDisp(0, linearSlide::down)
                .splineToSplineHeading(new Pose2d(loop_x+5,startY+5, downTangent), 0)
                .afterDisp(0, grabber::toggleClaw)
                .afterTime(1, linearSlide::up)
                .splineToSplineHeading(new Pose2d(Roriginal_x+4, subY, startHeading), 0)
                .afterDisp(0, linearSlide::score)
                .build();

        Action startToPark = drive.actionBuilder(new Pose2d(Roriginal_x+4, startY, startHeading))
                .afterDisp(0, linearSlide::down)
                .afterDisp(0, grabber::toggleClaw)
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

        Actions.runBlocking(new SequentialAction(startToSub, new InstantAction(grabber::toggleClaw), specCycleN, new InstantAction(grabber::toggleClaw), startToPark));


//        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
