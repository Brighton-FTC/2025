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



        RoadRunnerBotEntity Bot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity SpecBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity Bot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity SpecBot2 = new DefaultBotBuilder(meepMeep)
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


        Bot1.runAction(Bot1.getDrive().actionBuilder(new Pose2d(Roriginal_x, 56, -startHeading))
                        .splineTo(new Vector2d(30, 34), RsubHeading)
                .splineToConstantHeading(new Vector2d(loop_x, 10), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+5, 55, RsubHeading), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+10, 10, RsubHeading), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+15, 55, RsubHeading), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+20, 10, RsubHeading), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+25, 55, RsubHeading), RsubTangent)
                .build());

        SpecBot1.runAction(SpecBot1.getDrive().actionBuilder(new Pose2d(Loriginal_x, 56, -startHeading))
                .splineTo(new Vector2d(Loriginal_x, 34), LsubHeading)
                .lineToX(-30)//Faster than separate motion
                .splineToConstantHeading(new Vector2d(-loop_x, 10), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+5), 55, LsubHeading), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+10), 10, LsubHeading), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+15), 55, LsubHeading), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+20), 10, LsubHeading), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+25), 55, LsubHeading), LsubTangent)
                .build());
        Bot3.runAction(Bot3.getDrive().actionBuilder(new Pose2d(Loriginal_x, -56, startHeading))
                .splineTo(new Vector2d(-30, -34), LsubHeading)
                .splineToConstantHeading(new Vector2d(-1*(loop_x), -10), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+5), -55, LsubHeading), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+10), -10, LsubHeading), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+15), -55, LsubHeading), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+20), -10, LsubHeading), LsubTangent)
                .splineToSplineHeading(new Pose2d(-1*(loop_x+25), -55, LsubHeading), LsubTangent)
                .build());

        SpecBot2.runAction(SpecBot2.getDrive().actionBuilder(new Pose2d(Roriginal_x, -56, startHeading))
                .splineTo(new Vector2d(Roriginal_x, -34), RsubHeading)
                        .lineToX(30)//Faster than separate motion
                .splineToConstantHeading(new Vector2d(loop_x, -10), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+5, -55, RsubHeading), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+10, -10, RsubHeading), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+15, -55, RsubHeading), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+20, -10, RsubHeading), RsubTangent)
                .splineToSplineHeading(new Pose2d(loop_x+25, -55, RsubHeading), RsubTangent)
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Bot1)
                .addEntity(SpecBot1)
                .addEntity(Bot3)
                .addEntity(SpecBot2)
                .start();
    }
}
