package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RoadRunnerTopBTestSkeleton {
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
                .lineToX(38)
                .turn(Math.toRadians(90))
                .lineToY(38)

                .strafeTo(new Vector2d(48, 38))
                .strafeTo(new Vector2d(48, 33))
                //grab block
                .turn(Math.toRadians(134))
                .waitSeconds(2)
                //Lift???
                .strafeTo(new Vector2d(53, 53))
                .strafeTo(new Vector2d(56, 56))
                //drop 1st block
                .waitSeconds(2)
                .turn(Math.toRadians(-134))
                .strafeTo(new Vector2d(58, 38))
                .strafeTo(new Vector2d(58, 33))
                //grab block #2
                .turn(Math.toRadians(134))
                .waitSeconds(2)
                //Lift???
                .strafeTo(new Vector2d(53, 53))
                .strafeTo(new Vector2d(56, 56))
                                .waitSeconds(2)
                //exit and park
                .turn(Math.toRadians(-134))
                .strafeTo(new Vector2d(56, 38))
                .strafeTo(new Vector2d(-60, 38))
                .strafeTo(new Vector2d(-60, 58))
                                .waitSeconds(3)
                //TODO Ask team - Try for side block? Push in or shimmy out? Park or prepare to grab from middle? Any problems like hitting block re-work.

               // .strafeTo(new Vector2d(55, 10))
                //                .strafeTo(new Vector2d(55, 57))
                //                .strafeTo(new Vector2d(55, 10))
                //
                //                .strafeTo(new Vector2d(61, 10))
                //                .strafeTo(new Vector2d(62, 57))
                //                .strafeTo(new Vector2d(62, 38))
                //
                //                .strafeTo(new Vector2d(-60, 38))
                //                .strafeTo(new Vector2d(-60, 58))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
