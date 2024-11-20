package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {   // this code executes first in any Java application
        MeepMeep meepMeep = new MeepMeep(800);
        //OneBot(meepMeep);
        FourBots(meepMeep); // this is allowing both of the bots to run in the same meep meep environment
        double BucketDropLine = -50;
    }
/*
    private static void OneBot(MeepMeep meepMeep) { // declaring a class if their is only OneBot being on the Field
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep) // creating a bot named myBot
                // Set bot constraints: maxVel, max linear velocity, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -56, 90))
                        //.turn(Math.toRadians(90))

                .lineToY(0) // move 30 inches horizontally
                //.turn(Math.toRadians(90)) // turns robot 90 degrees clockwise
                //.lineToY(30) // Moves bot 30 inches vertically
                //.turn(Math.toRadians(90))
                //.lineToX(0)
                //.turn(Math.toRadians(90))
                //.lineToY(0)
                //        .waitSeconds(1) // waiting for one second
                //.turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT) // sets background to INTO THE DEEP
                .setDarkMode(false) // activates dark mode
                .setBackgroundAlpha(0.95f) // sets background transparency
                .addEntity(myBot) // opens the bot in the simulation environment
                .start();
    }*/

    private static void FourBots(MeepMeep meepMeep) { // declaring a class if their is two bots being on the Field
        //Declare our First bot
        RoadRunnerBotEntity redRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())                 // We set this bot to be blue
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redRight.runAction(redRight.getDrive().actionBuilder(new Pose2d(10, -56, Math.toRadians(-90)))
                .lineToY(-34)
                    .waitSeconds(1)
                .turn(Math.toRadians(90))
                .lineToX(40)
                .turn(Math.toRadians(25))
                    .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .lineToY(-50)
                                .waitSeconds(1)
                .splineTo(new Vector2d(50, -33), -50)
                    .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .lineToY(-50)
                    .waitSeconds(1)
                .turn(Math.toRadians(180))
                //.splineTo(new Vector2d(28, 8), 360)
                .build());

        // Declare our second bot
        RoadRunnerBotEntity redLeft = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        double BucketDropLine = -55;
        Action myAction = redLeft.getDrive().actionBuilder(new Pose2d(-16, -56, Math.toRadians(-90)))
                .lineToY(-38)
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .lineToX(-30)
                .turn(Math.toRadians(-25))
                .waitSeconds(1)
                .turn(Math.toRadians(-110))
                .lineToY(-56)
                .waitSeconds(1)
                .turn(Math.toRadians(60))
                .lineToY(-38)
                .waitSeconds(1)
                .lineToY(BucketDropLine)
                .turn(Math.toRadians(-60))
                .waitSeconds(1)
                .turn(Math.toRadians(45))
                .lineToY(-10)
                .turn(Math.toRadians(90))
                .lineToX(-26)
                .build();

        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        Action blueLeftAction = blueLeft.getDrive().actionBuilder(new Pose2d(-13, 60, Math.toRadians(90)))
                .lineToY(37)//Drop Preloaded
                    .waitSeconds(1)
                .turn(Math.toRadians(90))//turn towards spike mark
                .lineToX(-39)//drive towards spike mark 1
                .turn(Math.toRadians(25))//turn to pick up
                    .waitSeconds(1)//Picked up
                .turn(Math.toRadians(-85))//face bucket
                .lineToY(47)//drive bucket
                    .waitSeconds(1)//drop in
                .turn(Math.toRadians(125))//drive towards spike 3
                .lineToY(37)//pick up from spike 3
                    .waitSeconds(1)
                .turn(Math.toRadians(-155))
                .lineToY(50)
                    .waitSeconds(1)
                .lineToY(12)//drive back towards submersible
                .turn(Math.toRadians(90))//face towards park
                .lineToX(-26)//drive into park
                        .build();


        redLeft.runAction(myAction);
        blueLeft.runAction(blueLeftAction);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)  // sets background to INTO THE DEEP
                .setDarkMode(true) // activates dark mode
                .setBackgroundAlpha(1000f)  // sets background transparency

                // Add both of our declared bot entities
                .addEntity(redRight)  // opens the first bot in the meep meep visualizer
                .addEntity(redLeft) // opens the second bot in the meep meep visualizer
                .addEntity(blueLeft)
                .start();


    }
}