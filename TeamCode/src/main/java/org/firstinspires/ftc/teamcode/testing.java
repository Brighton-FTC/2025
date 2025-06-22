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




        Action cycle1 = drive.actionBuilder(new Pose2d(Roriginal_x, subY, startHeading))
                .splineToConstantHeading(new Vector2d(Roriginal_x, -60), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 11, -19), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 13, -60), startHeading)
                .build();

// Second spike
        Action cycle2 = drive.actionBuilder(new Pose2d(loop_x + 13, -60, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x + 14, -19), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 23, -60), startHeading)
                .build();

// Third spike
        Action cycle3 = drive.actionBuilder(new Pose2d(loop_x + 23, -60, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x + 24, -19), startHeading)
                .splineToConstantHeading(new Vector2d(loop_x + 33, -60), startHeading)
                .splineToSplineHeading(new Pose2d(loop_x+5, startY-20, downTangent), 0)
                .build();


        //Actions.runBlocking(testTurn);

        Actions.runBlocking(new SequentialAction(cycle1, cycle2, cycle3));

    }
}
