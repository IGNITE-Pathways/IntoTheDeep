package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBasket {
    public static void main(String[] args) {   // this code executes first in any Java application
        MeepMeep meepMeep = new MeepMeep(800);
        BlueOrRedBot(meepMeep); // this is allowing four bots to run in the same meep meep environment
    }

    private static void BlueOrRedBot(MeepMeep meepMeep) { // declaring a class if their is four bots being on the Field

        //declaring our third bot, blue left
        RoadRunnerBotEntity blueOrRed = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 20)
                .setDimensions(14, 16)
                .build();

        double startingXPosition = 8;
        double startingYPosition = 62;
        double moveRobotByInches = 26.5;
        // DEFINE X POSITIONS
        double firstSpecimenXPosition = 8;

        Pose2d beginPose = new Pose2d(startingXPosition, startingYPosition, Math.toRadians(-90));

        Action blueRightAction = blueOrRed.getDrive().actionBuilder(beginPose)
                .strafeTo(new Vector2d(firstSpecimenXPosition,startingYPosition - moveRobotByInches)) // drives to the chamber 35.5
                .waitSeconds(1)// drops preloaded specimen on the chamber

                // goes back so it doesn't hit the hitting the top right stand bar holding up the submersible
                .splineToLinearHeading(new Pose2d(startingXPosition, startingYPosition - moveRobotByInches + 4, Math.toRadians(170)), Math.toRadians(-90)) // goes back so it doesn't hit the hitting the top right stand bar holding up the submersible
                // moves in the direction of the sample and extendo extends
                //@ToDo: Move forward
                .strafeTo(new Vector2d(23,startingYPosition - moveRobotByInches)) //36.5
                .waitSeconds(2) // extendo takes in the sample

                // splines goes drop first sample in the high basket!
                //@ToDo: move closer
                .splineToLinearHeading(new Pose2d(53, 50, Math.toRadians(45)), Math.toRadians(90))
                .strafeTo(new Vector2d(46, 45)) //Move back
                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
                .splineToLinearHeading(new Pose2d(45, 9, Math.toRadians(180)), Math.toRadians(270)) // splines down to get ready to park
                .strafeTo(new Vector2d(21,9))  // splines to rung for level 1 ascent (3 points)
                .build();

        blueOrRed.runAction(blueRightAction);
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)  // sets background to INTO THE DEEP
                .setDarkMode(true) // activates dark mode
                .setBackgroundAlpha(1000f)  // sets background transparency
                .addEntity(blueOrRed)
                .start();

    }
}