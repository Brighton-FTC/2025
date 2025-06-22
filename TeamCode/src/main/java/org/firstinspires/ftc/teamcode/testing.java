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
@Autonomous(name = "trajectory test", preselectTeleOp = "Teleop")
public class testing extends LinearOpMode {
    public static double startY = -56;
    double startHeading = Math.toRadians(0);

    double downTangent = Math.toRadians(180);

    double Roriginal_x = 10;
    double Loriginal_x = -10;
    double loop_x = 39;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, startY, startHeading));




        Action startToSub = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .splineToConstantHeading(new Vector2d(Roriginal_x, -50), startHeading)
                .build();

        Action startToPark = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x+5,-55), startHeading)
                .build();



        waitForStart();


        Actions.runBlocking(startToSub);
//        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
