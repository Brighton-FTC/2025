package com.example.trajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep2 {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true"); // improve performance

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity Bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity SpecBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        double startHeading = Math.toRadians(90);

        double RsubTangent = Math.toRadians(90);
        double RsubHeading = Math.toRadians(0);
        double LsubHeading = Math.toRadians(180);
        double LsubTangent = Math.toRadians(-90);
        double Roriginal_x = 10;
        double Loriginal_x = -10;
        double loop_x = 39;



        Bot.runAction(Bot.getDrive().actionBuilder(new Pose2d(Loriginal_x, -56, startHeading))
                .splineTo(new Vector2d(Loriginal_x, -34), RsubHeading)
                .splineToConstantHeading(new Vector2d(-loop_x-5, -38), RsubTangent)
                .splineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(135)), RsubTangent)
                .splineToSplineHeading(new Pose2d(-loop_x-20, -38, Math.toRadians(0)), RsubTangent)
                .splineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(135)), RsubTangent)
                .splineToSplineHeading(new Pose2d(-57, -10, Math.toRadians(90)), RsubTangent)
                .splineToConstantHeading(new Vector2d(-65, -55), RsubTangent)
                .build());


        SpecBot.runAction(SpecBot.getDrive().actionBuilder(new Pose2d(Roriginal_x, -56, startHeading))
                // First cycle
                .splineTo(new Vector2d(Roriginal_x, -34), RsubHeading)
                .lineToX(30)
                .splineToConstantHeading(new Vector2d(loop_x, -10), RsubTangent)
                .splineToConstantHeading(new Vector2d(loop_x+5, -55), RsubTangent)
                .splineToConstantHeading(new Vector2d(Roriginal_x, -56), RsubTangent)
                // Second cycle
                .splineToConstantHeading(new Vector2d(Roriginal_x, -34), RsubHeading)
                .lineToX(30)
                .splineToConstantHeading(new Vector2d(loop_x+10, -10), RsubTangent)
                .splineToConstantHeading(new Vector2d(loop_x+15, -55), RsubTangent)
                .splineToConstantHeading(new Vector2d(Roriginal_x, -56), RsubTangent)
                // Third cycle
                .splineToConstantHeading(new Vector2d(Roriginal_x, -34), RsubHeading)
                .lineToX(30)
                .splineToConstantHeading(new Vector2d(loop_x+20, -10), RsubTangent)
                .splineToConstantHeading(new Vector2d(loop_x+25, -55), RsubTangent)
                .splineToConstantHeading(new Vector2d(Roriginal_x, -56), RsubTangent)
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Bot)
                .addEntity(SpecBot)
                .start();
    }
}
