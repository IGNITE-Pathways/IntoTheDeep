package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
    public static void main(String[] args) {   // this code executes first in any Java application
        MeepMeep meepMeep = new MeepMeep(1100);
        OneBot(meepMeep);
       // TwoBots(meepMeep); // this is allowing both of the bots to run in the same meep meep environment
    }

    private static void OneBot(MeepMeep meepMeep) { // declaring a class if their is only OneBot being on the Field
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep) // creating a bot named myBot
                // Set bot constraints: maxVel, max linear velocity, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 0, 0))
                .lineToX(30) // move 30 inches horizontally
                .turn(Math.toRadians(90)) // turns robot 90 degrees clockwise
                .lineToY(30) // Moves bot 30 inches vertically
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                        .waitSeconds(1) // waiting for one second
                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT) // sets background to INTO THE DEEP
                .setDarkMode(false) // activates dark mode
                .setBackgroundAlpha(0.95f) // sets background transparency
                .addEntity(myBot) // opens the bot in the simulation environment
                .start();
    }
    private static void TwoBots(MeepMeep meepMeep) { // declaring a class if their is two bots being on the Field
        //Declare our First bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())                 // We set this bot to be blue
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());

        // Declare our second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action myAction = mySecondBot.getDrive().actionBuilder(new Pose2d(30, 30, Math.toRadians(180)))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .build();

        mySecondBot.runAction(myAction);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)  // sets background to INTO THE DEEP
                .setDarkMode(true) // activates dark mode
                .setBackgroundAlpha(1000f)  // sets background transparency

                // Add both of our declared bot entities
                .addEntity(myFirstBot)  // opens the first bot in the meep meep visualizer
                .addEntity(mySecondBot) // opens the second bot in the meep meep visualizer
                .start();


    }
}