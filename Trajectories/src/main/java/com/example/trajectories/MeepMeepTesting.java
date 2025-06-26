package com.example.trajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double startY = -63;
    static double UP = Math.toRadians(90);
    static double LEFT = Math.toRadians(180);

    static double DOWN = Math.toRadians(270);
    static double RIGHT = Math.toRadians(360);

    static double startX = 9;
    static double subY = -31;
    static double loop_x = 37;

    static double SPLINE_Y_OFFSET = 0;
//    static double SPEC_COLLECT_OFFSET = 24; // 4
    static double scoreX = -5;
    static double collectY = -67;
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


//        double startHeading = Math.toRadians(90);
//
//        double downTangent = Math.toRadians(270);

//        double Roriginal_x = 10;
//        double Loriginal_x = -10;
//        double loop_x = 39;



//        Bot.runAction(Bot.getDrive().actionBuilder(new Pose2d(Loriginal_x, -56, startHeading))
//                .splineToConstantHeading(new Vector2d(Loriginal_x, -34), startHeading)
//                .splineToConstantHeading(new Vector2d(Loriginal_x, -40), startHeading)
//                .splineToConstantHeading(new Vector2d(-loop_x-9, -38), startHeading)
//                .splineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(225)), startHeading)
//                .splineToSplineHeading(new Pose2d(-loop_x-20, -38, Math.toRadians(90)), startHeading)
//                .splineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(225)), startHeading)
//                .splineToSplineHeading(new Pose2d(-57, -10, Math.toRadians(90)), startHeading)
//                .splineToConstantHeading(new Vector2d(-65, -55), startHeading)
//                .build());
//
//
//        SpecBot.runAction(SpecBot.getDrive().actionBuilder(new Pose2d(Roriginal_x, -56, startHeading))
//                // First cycle
//                .splineToConstantHeading(new Vector2d(Roriginal_x, -34), startHeading)
//                .splineToSplineHeading(new Pose2d(Roriginal_x, -40, startHeading), downTangent)
//                .splineToConstantHeading(new Vector2d(35, -40), startHeading)
//                .splineToConstantHeading(new Vector2d(loop_x, -10), startHeading)
//                .splineToConstantHeading(new Vector2d(loop_x+5, -55), startHeading)
//                .splineToConstantHeading(new Vector2d(loop_x+10, -10), startHeading)
//                .splineToConstantHeading(new Vector2d(loop_x+15, -55), startHeading)
//                .splineToConstantHeading(new Vector2d(loop_x+20, -10), startHeading)
//                .splineToConstantHeading(new Vector2d(loop_x+22, -55), startHeading)
//                .splineToSplineHeading(new Pose2d(loop_x+5,-55, downTangent), 90)
//                // Second cycle
//                .splineToSplineHeading(new Pose2d(Roriginal_x, -34, startHeading), 270)
//                .splineToSplineHeading(new Pose2d(loop_x+5,-55, downTangent), 90)
//                .splineToSplineHeading(new Pose2d(Roriginal_x, -34, startHeading), 270)
//                .splineToSplineHeading(new Pose2d(loop_x+5,-55, downTangent), 90)
//                .splineToSplineHeading(new Pose2d(Roriginal_x, -34, startHeading), 270)
//                .splineToConstantHeading(new Vector2d(loop_x+5,-55), startHeading)
//                .build());

        // TWO SPEC WORKING WITH FIXES
//        SpecBot.runAction(SpecBot.getDrive().actionBuilder(new Pose2d(startX, -56, UP))
//                        .splineToConstantHeading(new Vector2d(startX, subY), UP) // score
//                        .waitSeconds(1)
//                        .setTangent(DOWN)
//                        .splineToLinearHeading(new Pose2d(35, startY-SPLINE_Y_OFFSET+SPEC_COLLECT_OFFSET, DOWN), DOWN)
//                        .waitSeconds(1)
//                        .setTangent(UP)
//                        .splineToSplineHeading(new Pose2d(startX+2, subY-1, UP), UP) // score
//                        .waitSeconds(1)
//                        .setTangent(DOWN)
//                        .splineToConstantHeading(new Vector2d(loop_x,startY+2), DOWN) // park
//                        .build());

        // REMEMBER: setTanget changes tanget at start, tanget parameter changes tanget at end

        double sampCollectTangent = Math.toRadians(320);

        // FIVE SPEC
        SpecBot.runAction(SpecBot.getDrive().actionBuilder(new Pose2d(startX, startY, UP))
                        .splineToConstantHeading(new Vector2d(scoreX, subY), UP) // score
                        .waitSeconds(1)
                        .setTangent(Math.toRadians(315))
                        .splineToLinearHeading(new Pose2d(35, collectY, DOWN), DOWN)
                        .setTangent(UP)
                        .splineToSplineHeading(new Pose2d(scoreX+4, subY-SPLINE_Y_OFFSET, UP), UP) // score

                        // COLLECT THE 3 SAMPLES

                        .setTangent(Math.toRadians(315))
                        .splineToConstantHeading(new Vector2d((loop_x + scoreX + 1) / 4 * 3, (-40 + subY ) / 2), UP)
                        .splineToConstantHeading(new Vector2d(loop_x, -10), UP)
                        .setTangent(sampCollectTangent)
                        .splineToConstantHeading(new Vector2d(loop_x+7, -55), UP)
                        .setTangent(UP)
                        .splineToConstantHeading(new Vector2d(loop_x+10, -10), UP)
                        .setTangent(sampCollectTangent)
                        .splineToConstantHeading(new Vector2d(loop_x+17, -55), UP)
                        .setTangent(UP)
                        .splineToConstantHeading(new Vector2d(loop_x+27, -10), RIGHT)
                        .setTangent(DOWN)
                        .splineToSplineHeading(new Pose2d(loop_x+27, -55-SPLINE_Y_OFFSET, UP), DOWN)
                        .waitSeconds(1)

                        // SCORE THE 3 SPEC

                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(35, collectY-SPLINE_Y_OFFSET, DOWN), DOWN)
                        .setTangent(UP)
                        .splineToSplineHeading(new Pose2d(scoreX+8, subY-SPLINE_Y_OFFSET, UP), UP) // score
                        .waitSeconds(1)
                        .setTangent(Math.toRadians(315))
                        .splineToLinearHeading(new Pose2d(35, collectY-SPLINE_Y_OFFSET, DOWN), DOWN)
                        .setTangent(UP)
                        .splineToSplineHeading(new Pose2d(scoreX+12, subY-SPLINE_Y_OFFSET, UP), UP) // score
                        .waitSeconds(1)
                        .setTangent(Math.toRadians(315))
                        .splineToLinearHeading(new Pose2d(35, collectY-SPLINE_Y_OFFSET, DOWN), DOWN)
                        .setTangent(UP)
                        .splineToSplineHeading(new Pose2d(scoreX+16, subY-SPLINE_Y_OFFSET, UP), UP) // score
                        .waitSeconds(1)

                        // PARK

                        .setTangent(DOWN)
                        .splineToConstantHeading(new Vector2d(loop_x,collectY-SPLINE_Y_OFFSET), DOWN) // park
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Bot)
                .addEntity(SpecBot)
                .start();
    }
}
