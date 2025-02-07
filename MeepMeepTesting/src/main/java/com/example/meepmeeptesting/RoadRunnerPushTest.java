package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RoadRunnerPushTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        //Edit Below
        //Starting Position
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 60, Math.toRadians(180)))
               //Edit Path
                .lineToX(56)
                        .waitSeconds(0.5)
                .turn(Math.toRadians(0))
                .lineToX(38)
                .turn(Math.toRadians(90))
                .lineToY(10)

                .strafeTo(new Vector2d(47, 10))
                .strafeTo(new Vector2d(47, 57))
                .strafeTo(new Vector2d(47, 10))

                .strafeTo(new Vector2d(55, 10))
                .strafeTo(new Vector2d(55, 57))
                .strafeTo(new Vector2d(55, 10))

                .strafeTo(new Vector2d(61, 10))
                .strafeTo(new Vector2d(62, 57))
                .strafeTo(new Vector2d(62, 38))

                .strafeTo(new Vector2d(-60, 38))
                .strafeTo(new Vector2d(-60, 58))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
