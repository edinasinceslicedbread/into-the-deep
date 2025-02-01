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
                //20 Point autonomous
                        .waitSeconds(3)
                .lineToX(55)
                .lineToX(38)
                .turn(Math.toRadians(90))
                .lineToY(38)

                .strafeTo(new Vector2d(48, 38))
                .strafeTo(new Vector2d(48, 33))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(48, 38))
                //grab block #1
                .turn(Math.toRadians(134))
                .waitSeconds(2)
                //Lift???
                .strafeTo(new Vector2d(53, 53))
                .strafeTo(new Vector2d(56, 56))
                .waitSeconds(2)
                //drop 1st block
                .strafeTo(new Vector2d(53, 53))
                .waitSeconds(2)
                //drop 1st block
                .waitSeconds(2)
                .turn(Math.toRadians(-134))
                .strafeTo(new Vector2d(58, 38))
                .strafeTo(new Vector2d(58, 33))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(58, 38))
                //grab block #2
                .turn(Math.toRadians(134))
                .waitSeconds(2)
                //Lift???
                .strafeTo(new Vector2d(53, 53))
                .strafeTo(new Vector2d(56, 56))
                .waitSeconds(2)
                .strafeTo(new Vector2d(53, 53))
                .waitSeconds(2)
                //Drop 2nd block
                .strafeTo(new Vector2d(50, 50))
                .turn(Math.toRadians(46))
                .strafeTo(new Vector2d(50, 10))
                .strafeTo(new Vector2d(61, 10))
                .strafeTo(new Vector2d(61, 50))
                                .tangen
                // Alternative preparation for park
                //.strafeTo(new Vector2d(61, 38))
                //.strafeTo(new Vector2d(-61, 38))
                //.strafeTo(new Vector2d(-61, 58))
               // .waitSeconds(3)
                //TODO Pick which one ^ or v
                //alternative preparation for pick from middle
                //.strafeTo(new Vector2d(61, 10))
                //.turn(Math.toRadians(90))
                //.waitSeconds(3)

                //TODO Ask team:
                // .
                // Push Block into the under basket zone.
                // .
                // Park or prepare to grab from middle?
                // Any problems like hitting block re-work.
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
