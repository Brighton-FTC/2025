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
    public static double startHeading = Math.toRadians(90);

    public static double downTangent = Math.toRadians(270);

    public static double Roriginal_x = 10;
    public static double Loriginal_x = -10;
    public static double loop_x = 36;

    public static double subY = -38;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, startY, startHeading));




        Action startToSub = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .splineToConstantHeading(new Vector2d(Roriginal_x, subY), startHeading)
                .build();

        Action subToPark = drive.actionBuilder(new Pose2d(Roriginal_x, subY, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x,startY), startHeading)
                .build();

        Action specCycle1 = drive.actionBuilder(new Pose2d(loop_x+5, startY, startHeading))
                .splineToConstantHeading(new Vector2d(Roriginal_x, subY), startHeading)
                .build();

        //just repeat this for the rest of the spec cycle
        Action specCycleN = drive.actionBuilder(new Pose2d(Roriginal_x, -34, startHeading))
                .splineToSplineHeading(new Pose2d(loop_x+5,startY+5, downTangent), 0)
                .splineToSplineHeading(new Pose2d(Roriginal_x, subY, startHeading), 0)
                .build();

        Action cycle1 = drive.actionBuilder(new Pose2d(Roriginal_x, subY, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x + 10, -48), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 10, -19), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 12, -65), startHeading)
                .build();

// Second spike
        Action cycle2 = drive.actionBuilder(new Pose2d(loop_x + 12, -65, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x + 19, -19), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 24, -65), startHeading)
                .build();

// Third spike
        Action cycle3 = drive.actionBuilder(new Pose2d(loop_x + 24, -65, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x + 31, -19), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 36, -65), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x+5, startY), startHeading)
                .build();


        Action testTurn = drive.actionBuilder(new Pose2d(0, 0 , startHeading)).turnTo(downTangent).build();
        waitForStart();

        //Actions.runBlocking(testTurn);

        Actions.runBlocking(new SequentialAction(startToSub, new SleepAction(0.1), cycle1, cycle2, cycle3, specCycle1));

    }
}
