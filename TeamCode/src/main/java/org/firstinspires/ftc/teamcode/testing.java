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
@Autonomous(name = "trajectory test", preselectTeleOp = "Teleop")
public class testing extends LinearOpMode {
    public static double startY = -72;
    double startHeading = Math.toRadians(90);

    double downTangent = Math.toRadians(270);

    double Roriginal_x = 10;
    double Loriginal_x = -10;
    double loop_x = 26;

    double subY = -38;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, startY, startHeading));




        Action startToSub = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .splineToConstantHeading(new Vector2d(Roriginal_x, subY), startHeading)
                .build();

        Action subToPark = drive.actionBuilder(new Pose2d(Roriginal_x, subY, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x,startY+7), startHeading)
                .build();

        Action specCycle1 = drive.actionBuilder(new Pose2d(loop_x+5, startY+7, startHeading))
                .splineToSplineHeading(new Pose2d(Roriginal_x, -34, startHeading), 0)
                .build();

        Action specCycle2 = drive.actionBuilder(new Pose2d(Roriginal_x, -34, startHeading))
                .splineToSplineHeading(new Pose2d(loop_x+5,startY+7, downTangent), 0)
                .splineToSplineHeading(new Pose2d(Roriginal_x, -34, startHeading), 0)
                .build();

        Action subToSpike = drive.actionBuilder(new Pose2d(Roriginal_x, subY, startHeading))
                .splineToSplineHeading(new Pose2d(Roriginal_x, -48, startHeading), downTangent)
                .splineToConstantHeading(new Vector2d(36, -48), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x, -10), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+5, startY+7), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+10, -10), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+15, startY+7), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+20, -10), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+22, startY+7), startHeading)
                .splineToSplineHeading(new Pose2d(loop_x+5,startY+7, downTangent), 0)
                .build();



        waitForStart();

        Actions.runBlocking(new SequentialAction(startToSub, new SleepAction(1), specCycle2));

    }
}
