package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Timer;

public class MeepMeepTesting {
    public static void main(String[] args) {   // this code executes first in any Java application
        MeepMeep meepMeep = new MeepMeep(900);
        FourBots(meepMeep); // this is allowing four bots to run in the same meep meep environment
      //  double RedBucketDropLine = -50;
        ;
    }

    private static void FourBots(MeepMeep meepMeep) { // declaring a class if their is four bots being on the Field

        //Declare our First bot, red right
        RoadRunnerBotEntity redRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())                 // We set this bot to be blue
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        // defining the actions of red right
        Action redRightAction = redRight.getDrive().actionBuilder(new Pose2d(10, -56, Math.toRadians(-90)))
                .lineToY(-37)//Drop Preloaded specimen on the chamber
                .waitSeconds(1)
                .turn(Math.toRadians(90))//turn towards spike mark
                .lineToX(39)//drive towards spike mark 1
                .turn(Math.toRadians(25))//turn to pick up
                .waitSeconds(1)//Picked up
                .turn(Math.toRadians(-85))//face bucket
                .lineToY(-47)//drive bucket
                .waitSeconds(1)//drop in
                .turn(Math.toRadians(125))//drive towards spike 3
                .lineToY(-37)//pick up from spike 3
                .waitSeconds(1)
                .turn(Math.toRadians(-155))
                .lineToY(-50)
                .waitSeconds(1)
                .lineToY(-12)//drive back towards submersible
                .turn(Math.toRadians(90))//face towards park
                .lineToX(26)//drive into park
                .build();

        // Declare our second bot, red left
        RoadRunnerBotEntity redLeft = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        double BucketDropLine = -55;  // defining the actions of red left
        Action redLeftAction = redLeft.getDrive().actionBuilder(new Pose2d(-16, -56, Math.toRadians(-90)))
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
 //declaring our third bot, blue left
        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180),Math.toRadians(180), 15)
                .build();

        // defining the actions of the bot, blue left
        Action blueLeftAction = blueLeft.getDrive().actionBuilder(new Pose2d(-13, 60, Math.toRadians(270)))
                .splineTo(new Vector2d(-7,33), Math.toRadians(270)) // drops specimen on the chamber
                .lineToY(38)
                .turn(Math.toRadians(-70)) // turns to get a sample
                .lineToX(-23) // reaches for sample with extendo
                .waitSeconds(0.7)
                .turn(Math.toRadians(-75)) // turns
                .lineToY(45)// drops  1st sample in observation zone
                .waitSeconds(0.5) // take 0.5 seconds to do that
                .splineToLinearHeading(new Pose2d(-38,38, Math.toRadians(210)), Math.toRadians(90)) // gets 2nd sample while human player puts clip on 1st one
                .waitSeconds(0.7) // waits 0.7 seconds to do that
                .splineTo(new Vector2d(-37,40), )
                .lineToY(40) // goes a little back
                .turn(Math.toRadians(-80)) // turns to observation zone
                .waitSeconds(0.5) // waits
                .lineToX(-37) // gets the sample
                .turn(Math.toRadians(-85))
                //.lineToY(-39) //drive towards spike mark 1
                // .waitSeconds(200)
                .build();
//                .lineToX(-39)//drive towards spike mark 1
//                .turn(Math.toRadians(25))//turn to pick up
//                    .waitSeconds(1)//Picked up
//                .turn(Math.toRadians(-85))//face bucket
//                .lineToY(47)//drive bucket
//                    .waitSeconds(1)//drop in
//                .turn(Math.toRadians(125))//drive towards spike 3
//                .lineToY(37)//pick up from spike 3
//                    .waitSeconds(1)
//                .turn(Math.toRadians(-155))
//                .lineToY(50)
//                    .waitSeconds(1)
//                .lineToY(12)//drive back towards submersible
//                .turn(Math.toRadians(90))//face towards park
//                .lineToX(-26)//drive into park
//                        .build();

// declaring our fourth bot blue right
        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // defining the action of our next bot, blue right
        Action blueRightAction = blueLeft.getDrive().actionBuilder(new Pose2d(13, 60, Math.toRadians(90)))
                .lineToY(38)
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .lineToX(30)
                .turn(Math.toRadians(-25))
                .waitSeconds(1)
                .turn(Math.toRadians(-110))
                .lineToY(56)
                .waitSeconds(1)
                .turn(Math.toRadians(60))
                .lineToY(38)
                .waitSeconds(1)
                .lineToY(50)
                .turn(Math.toRadians(-60))
                .waitSeconds(1)
                .turn(Math.toRadians(45))
                .lineToY(10)
                .turn(Math.toRadians(90))
                .lineToX(26)
                .build();

        redRight.runAction(redRightAction);
        redLeft.runAction(redLeftAction);
        blueLeft.runAction(blueLeftAction);
        blueRight.runAction(blueRightAction);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)  // sets background to INTO THE DEEP
                .setDarkMode(true) // activates dark mode
                .setBackgroundAlpha(1000f)  // sets background transparency

                // Add both of our declared bot entities
//                .addEntity(redRight)  // opens the first bot in the meep meep visualizer
//             .addEntity(redLeft) // opens the second bot in the meep meep visualizer
              .addEntity(blueLeft)
//                .addEntity(blueRight)
                .start();


    }
}