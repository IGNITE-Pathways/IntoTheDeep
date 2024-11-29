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
        MeepMeep meepMeep = new MeepMeep(800);
        FourBots(meepMeep); // this is allowing four bots to run in the same meep meep environment
        double RedBucketDropLine = -50;
        ;
    }

    private static void FourBots(MeepMeep meepMeep) { // declaring a class if their is four bots being on the Field

        //Declare our First bot, red right
        RoadRunnerBotEntity redRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())                 // We set this bot to be blue
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,14)
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
                .setDimensions(14,14)
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
                .setConstraints(70, 70, Math.toRadians(180),Math.toRadians(180), 20)
                .setDimensions(14,14)

                .build();

        // defining the actions of the bot, blue left
        Action blueLeftAction = blueLeft.getDrive().actionBuilder(new Pose2d(-13, 63, Math.toRadians(90)))
                .strafeTo(new Vector2d(-2,31)) // drives to the chamber
                .waitSeconds(1)// drops preloaded first specimen on the chamber
                .strafeTo(new Vector2d(-31,34)) // drives to the left
                .strafeTo((new Vector2d(-33,28))) // gets ready to do a nice spline, without hitting the top left stand bar holding up the submersible
                .splineToLinearHeading(new Pose2d(-48,15, Math.toRadians(270)), Math.toRadians(75)) // splines to the first sample
                .strafeTo(new Vector2d(-48,52)) // pushes 1st sample into to the observation zone
                .strafeTo(new Vector2d(-45,35)) // strafes a little down and right to make a smooth spline
                .splineToConstantHeading(new Vector2d(-58,15), Math.toRadians(180))  // goes to second sample
                .strafeTo(new Vector2d(-58,52)) // pushes 2nd sample into the observation zone
                .splineToConstantHeading(new Vector2d(-34,60), Math.toRadians(90)) // splines to the edge of observation zone to get specimen made by human player
                .waitSeconds(0.3) // waits a little to open the claw
                .lineToY(63) // moves forward
                .waitSeconds(0.5)  // hooks the specimen in the claw and moves viper slides up
                .splineToLinearHeading(new Pose2d(-4, 31, Math.toRadians(90)), Math.toRadians(270)) // splines to chamber to hook second specimen
                .waitSeconds(1)// drops second specimen on the chamber
                .strafeTo(new Vector2d(-32,34)) // drives to the left
                .splineToLinearHeading(new Pose2d(-64,15, Math.toRadians(270)), Math.toRadians(180)) // splines to third sample
                .strafeTo(new Vector2d(-64,52)) // pushes it to the observation zone
                .splineToConstantHeading(new Vector2d(-34,60), Math.toRadians(90)) // splines to the edge of observation zone to get third specimen put by the human player
                .waitSeconds(0.1) // waits a little to open the claw
                .lineToY(63) // moves forward
                .waitSeconds(0.5)  // hooks the specimen in the claw and moves viper slides up
                .splineToLinearHeading(new Pose2d(-6, 31, Math.toRadians(90)), Math.toRadians(270)) // splines to chamber to hook third specimen
                .waitSeconds(1)// puts third specimen on the chamber
                .splineToLinearHeading(new Pose2d(-34,60, Math.toRadians(270)), Math.toRadians(60)) // splines to the edge of observation zone to get fourth specimen made by human player
                .waitSeconds(0.1) // waits a little to open the claw
                .lineToY(63) // moves forward
                .waitSeconds(0.5)  // hooks the fourth specimen in the claw and moves viper slides up
                .lineToY(60) // moves a little down for a clean spline to hook fourth the specimen
                .splineToLinearHeading(new Pose2d(-8, 31, Math.toRadians(90)), Math.toRadians(270)) // splines to chamber to hook fourth specimen
                .waitSeconds(1)// drops specimen on the chamber
                .strafeTo(new Vector2d(-47,65))

                .build();

// declaring our fourth bot blue right
        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,14)
                .build();

        // defining the action of our next bot, blue right
        Action blueRightAction = blueLeft.getDrive().actionBuilder(new Pose2d(13, 63, Math.toRadians(90)))
                .strafeTo(new Vector2d(12.5,31)) // drives to the chamber
                .waitSeconds(1)// drops preloaded specimen on the chamber
                .splineToLinearHeading(new Pose2d(12.5, 37, Math.toRadians(345)), Math.toRadians(90)) // goes back so it doesn't hit the hitting the top right stand bar holding up the submersible
                .strafeTo(new Vector2d(26,32)) // moves in the direction of the sample and extendo extends
                .waitSeconds(1) // extendo takes in the sample
                .strafeTo(new Vector2d(26,34)) // goes back so it doesn't hit the hitting the top right stand bar holding up the submersible
                .splineToLinearHeading(new Pose2d(55, 56, Math.toRadians(-135)), Math.toRadians(90)) // splines goes drop first sample in the high basket!
                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
                .splineToLinearHeading(new Pose2d(33, 26, Math.toRadians(360)), Math.toRadians(270)) // robot aligns itself to get the second sample
                .waitSeconds(2) // extendo extends and takes in the second sample
                .splineToLinearHeading(new Pose2d(55, 56, Math.toRadians(-135)), Math.toRadians(90)) // splines goes to drop second sample in the high basket!
                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
                .splineToLinearHeading(new Pose2d(47, 26, Math.toRadians(360)), Math.toRadians(270)) // robot aligns itself to get the third sample
                .waitSeconds(2) // extendo extends and takes in the third sample
                .splineToLinearHeading(new Pose2d(55, 56, Math.toRadians(-135)), Math.toRadians(0)) // splines goes to drop third sample in the high basket!
                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
                .splineToLinearHeading(new Pose2d(21.5, 10, Math.toRadians(360)), Math.toRadians(-180)) // splines to rung for level 1 ascent (3 points)

                .build();

        redRight.runAction(redRightAction);
        redLeft.runAction(redLeftAction);
        blueLeft.runAction(blueLeftAction);
        blueRight.runAction(blueRightAction);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)  // sets background to INTO THE DEEP
                .setDarkMode(true) // activates dark mode
                .setBackgroundAlpha(1000f)  // sets background transparency

                // Add both of our declared bot entities
                .addEntity(redRight)  // opens the first bot in the meep meep visualizer
                .addEntity(redLeft) // opens the second bot in the meep meep visualizer
              .addEntity(blueLeft)
                .addEntity(blueRight)
                .start();


    }
}