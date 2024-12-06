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

        double startingXPosition = -8;
        double startingYPosition = 62;
        double moveRobotByInches = 26.5;

        Pose2d beginPose = new Pose2d(startingXPosition, startingYPosition, Math.toRadians(-90));

        // DEFINE X POSITIONS
        double firstSpecimenXPosition = -2;
        double secondSpecimenXPosition = -5;
        double thirdSpecimenXPosition = -6;
        double obsZoneXPosition = -44;
        double obsZoneSpecimenPickupYPositionError = 2;

        // defining the actions of the bot, blue left, THIS IS THE OLD ONE
        Action blueOrRedObsZone = blueOrRed.getDrive().actionBuilder(beginPose)
                // moveToDropPreloadedSpecimen
                .strafeTo(new Vector2d(firstSpecimenXPosition, startingYPosition - moveRobotByInches)) //Y=35
                .waitSeconds(2)

                // dragSampleFromSpikeMarkToObsZone
                .strafeTo(new Vector2d(-34, startingYPosition - moveRobotByInches + 4)) //Y=39
                .strafeTo((new Vector2d(-34, startingYPosition - moveRobotByInches - 15))) //Y=20
                .splineToLinearHeading(new Pose2d(obsZoneXPosition, startingYPosition - 52, Math.toRadians(90)), Math.toRadians(90)) //Y=10
                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 9)) //Y=54
                .waitSeconds(.1)
                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 16)) //Y=48
                .waitSeconds(.1)
                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError - 0.5)) //Y=60
                .waitSeconds(0.3) //PICK SPECIMEN FROM WALL

                // sequence1: DONE

                // moveToDropSecondSpecimen
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
                .strafeTo(new Vector2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches)) //Y=35
                .waitSeconds(2)

                // splineToPickThirdSpecimenFromObservationZone
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(obsZoneXPosition, startingYPosition - 8, Math.toRadians(90)), Math.toRadians(90)) //Y=54
                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError + 1.5)) //Y=61
                .waitSeconds(0.3)

                // sequence2: DONE

                // moveToDropThirdSpecimen
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
                .strafeTo(new Vector2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches - 1.5)) //Y=35
                .waitSeconds(2)

                // sequence3: DONE

                // parking
                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 5 )) //Y=61

                // sequence4: DONE
                .build();

        blueOrRed.runAction(blueOrRedObsZone);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)  // sets background to INTO THE DEEP
                .setDarkMode(true) // activates dark mode
                .setBackgroundAlpha(1000f)  // sets background transparency
                .addEntity(blueOrRed)
                .start();

    }
}