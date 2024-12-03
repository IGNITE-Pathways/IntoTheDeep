package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingObsZone {
    public static void main(String[] args) {   // this code executes first in any Java application
        MeepMeep meepMeep = new MeepMeep(800);
        BlueBot(meepMeep); // this is allowing four bots to run in the same meep meep environment
        double RedBucketDropLine = -50;
    }

    private static void BlueBot(MeepMeep meepMeep) { // declaring a class if their is four bots being on the Field

        //declaring our third bot, blue left
        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 20)
                .setDimensions(14, 16)
                .build();

        int startingYPosition = 62;
        int moveRobotByInches = 28;
        int firstSpecimenXPosition = -2;
        int secondSpecimenXPosition = -6;
        int thirdSpecimenXPosition = -10;
        int parkingXPosition = -44;

        // defining the actions of the bot, blue left, THIS IS THE OLD ONE
        Action blueObsZone = blueLeft.getDrive().actionBuilder(new Pose2d(-8, startingYPosition, Math.toRadians(-90)))
                // moveToDropPreloadedSpecimen
                .strafeTo(new Vector2d(firstSpecimenXPosition, startingYPosition - moveRobotByInches)) //Y=35
                .waitSeconds(2)

                // dragSampleFromSpikeMarkToObsZone
//                .strafeTo(new Vector2d(-34, startingYPosition - moveRobotByInches + 4)) //Y=39
//                .strafeTo((new Vector2d(-34, startingYPosition - moveRobotByInches - 7))) //Y=28
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-34, startingYPosition - moveRobotByInches - 7, Math.toRadians(0)), Math.toRadians(-90)) //Y=29
                .splineToLinearHeading(new Pose2d(-46, startingYPosition - 52, Math.toRadians(90)), Math.toRadians(90)) //Y=10
                .strafeTo(new Vector2d(-46, startingYPosition - 1)) //Y=61
                .waitSeconds(0.3) //PICK SPECIMEN FROM WALL

                // sequence1: DONE

                // moveToDropSecondSpecimen
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
                .strafeTo(new Vector2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches)) //Y=35
                .waitSeconds(2)

                // splineToPickThirdSpecimenFromObservationZone
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-46, startingYPosition - 8, Math.toRadians(90)), Math.toRadians(90)) //Y=54
                .strafeTo(new Vector2d(-46, startingYPosition - 1)) //Y=61
                .waitSeconds(0.3)

                // sequence2: DONE

                // moveToDropThirdSpecimen
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
                .strafeTo(new Vector2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches)) //Y=35
                .waitSeconds(2)

                // sequence3: DONE

                // parking
                .strafeTo(new Vector2d(parkingXPosition, startingYPosition - 1 )) //Y=61

                // sequence4: DONE
                .build();

        blueLeft.runAction(blueObsZone);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)  // sets background to INTO THE DEEP
                .setDarkMode(true) // activates dark mode
                .setBackgroundAlpha(1000f)  // sets background transparency
                .addEntity(blueLeft)
                .start();

    }
}