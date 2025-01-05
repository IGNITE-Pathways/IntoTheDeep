//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous(name = "Auto Red Obs Zone", group = "Linear OpMode")
//public class AutoRedObservationZone extends LinearOpMode {
//
//    protected final ElapsedTime runtime = new ElapsedTime();
//    SequentialAction sequence1 = null;
//    SequentialAction sequence2 = null;
//    SequentialAction sequence3 = null;
//    SequentialAction sequence4 = null;
//
//    protected boolean pick2Samples = true;
//
//    @Override
//    public void runOpMode() {
//
//        DcMotor leftFrontDrive = hardwareMap.get(DcMotorEx.class, "rightback"); //rightback
//        DcMotor leftBackDrive = hardwareMap.get(DcMotorEx.class, "rightfront"); //rightfront`
//        DcMotor rightBackDrive = hardwareMap.get(DcMotorEx.class, "leftfront"); //leftfront
//        DcMotor rightFrontDrive = hardwareMap.get(DcMotorEx.class, "leftback"); //leftback
//        DigitalChannel greenLED = hardwareMap.get(DigitalChannel.class, "greenled"); //6
//        DigitalChannel redLED = hardwareMap.get(DigitalChannel.class, "redled"); //7
//
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
//        redLED.setMode(DigitalChannel.Mode.OUTPUT);
//
//        Viper viper = new Viper(hardwareMap);
//        Extendo extendo = new Extendo(hardwareMap);
//        viper.initialize();
//        extendo.initialize();
//        greenLED.setState(true); //SPECIMEN ONLY
//        redLED.setState(false);
//
//        // GAME FIELD CAN BE DIFFERENT BECAUSE OF HOW IT IS ASSEMBLED OR HOW THE MATS ARE PLACED / CUT
//        double startingXPosition = -8;
//        double startingYPosition = 62;
//        double moveRobotByInches = 26.4;
//
//        Pose2d beginPose = new Pose2d(startingXPosition, startingYPosition, Math.toRadians(-90));
//        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
//
//        // DEFINE X POSITIONS
//        double firstSpecimenXPosition = -2;
//        double secondSpecimenXPosition = -5;
//        double thirdSpecimenXPosition = -7;
//        double obsZoneXPosition = -44;
//        double obsZoneSpecimenPickupYPositionError = 2;
//
//        Action moveToDropFirstSpecimen = drive.actionBuilder(beginPose)
//                .strafeTo(new Vector2d(firstSpecimenXPosition, startingYPosition - moveRobotByInches))
//                .build();
//
//        Action dragSampleFromSpikeMarkToObsZone =
//                drive.actionBuilder(new Pose2d(firstSpecimenXPosition, startingYPosition - moveRobotByInches, Math.toRadians(-90))) //35
//                        .strafeTo(new Vector2d(-34, startingYPosition - moveRobotByInches + 4)) //Y=39
//                        .strafeTo((new Vector2d(-34, startingYPosition - moveRobotByInches - 15))) //Y=20
//                        .splineToLinearHeading(new Pose2d(obsZoneXPosition, startingYPosition - 52, Math.toRadians(90)), Math.toRadians(90)) //Y=10
//                        .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 9)) //Y=53
//                        .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 17)) //Y=45 -- go back
//                        //@TODO: ADD DELAY So Human Player can clip Sample to make a Specimen
//                        .waitSeconds(pick2Samples ? 4: 0)
//                        .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError + 0.5)) //Y=60 >> Moving 50 inches straight
//                        .build();
//
//        sequence1 = new SequentialAction(
//                viper.getReadyToDropSpecimen(),
//                new SleepAction(3.0),
//                moveToDropFirstSpecimen,
//                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
//                viper.openClaw(),
//                viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN),
//                dragSampleFromSpikeMarkToObsZone);
//
//        Action moveBackAndPickSpecimentAtTheSameTime = drive.actionBuilder(new Pose2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError + 0.5, Math.toRadians(90)))
//                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 4)) //Y=48
//                .afterDisp(.1, viper.getReadyToDropSpecimen()) // viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN + 4)
//                .build();
//
//        Action moveToDropSecondSpecimen = drive.actionBuilder(new Pose2d(obsZoneXPosition, startingYPosition - 4, Math.toRadians(90)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
//                .strafeTo(new Vector2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches)) //Y=35
//                .build();
//
//        //If pick2Samples = false
//        Action splineToPickThirdSpecimenFromObservationZone = drive.actionBuilder(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches, Math.toRadians(-90)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(obsZoneXPosition, startingYPosition - 8, Math.toRadians(90)), Math.toRadians(90)) //Y=54
//                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError + 1.5))
//                .build();
//
//        //If pick2Samples = true
//        Action splineToPickSampleFromObservationZoneAndPark = drive.actionBuilder(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches, Math.toRadians(-90)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-38, startingYPosition -6 , Math.toRadians(0)), Math.toRadians(180)) //Y=54
//                .build();
//
//        sequence2 = new SequentialAction(
//                viper.closeClaw(), new SleepAction(.2), //Grabs Specimen from the wall
//                moveBackAndPickSpecimentAtTheSameTime,
//                moveToDropSecondSpecimen,
//                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
//                viper.openClaw(),
//                viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN));
//
//        Action moveBackAndPickSpecimentAtTheSameTime1 = drive.actionBuilder(new Pose2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError + 1.5, Math.toRadians(90)))
//                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 5)) //Y=48
//                .afterDisp(.1, viper.getReadyToDropSpecimen()) // viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN + 4)
//                .waitSeconds(.1)
//                .build();
//
//        Action moveToDropThirdSpecimen = drive.actionBuilder(new Pose2d(obsZoneXPosition, startingYPosition - 5, Math.toRadians(90)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
//                .afterDisp(.1, viper.getReadyToDropSpecimen())
//                .strafeTo(new Vector2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches - 1.5)) //Y=35
//                .build();
//
//        sequence3 = new SequentialAction(
//                viper.closeClaw(), new SleepAction(.2), //Grabs Specimen from the wall
//                moveBackAndPickSpecimentAtTheSameTime1,
//                moveToDropThirdSpecimen,
//                extendo.elbowDown(),
//                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
//                viper.openClaw());
//
//        Action parkingAction = drive.actionBuilder(new Pose2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches - 1.5, Math.toRadians(-90)))
//                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 5)) //Y=57
//                .build();
//
//        sequence4 = new SequentialAction(viper.closeClaw(), viper.driveToPositionInInches(XBot.VIPER_HOME), parkingAction);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//        runtime.reset();
//
//        ///////////////////////////////////////////////////////////////////////////////
//
//        //sequence1 = 1st Specimen dropped, 2nd Specimen in Obs Zone, Robot in Obs Zone
//        Actions.runBlocking(sequence1);
//        telemetry.addData("First Sequence:", runtime.seconds());
//
//        Actions.runBlocking(viper.closeClaw());
//
//        //sequence2 = 2nd Specimen dropped, Robot in Obs Zone
//        Actions.runBlocking(sequence2);
//        telemetry.addData("Second Sequence:", runtime.seconds());
//
//        if (pick2Samples) {
//            Actions.runBlocking(new SequentialAction(
//                    splineToPickSampleFromObservationZoneAndPark,
//                    viper.driveToPositionInInches(XBot.VIPER_HOME),
//                    extendo.elbowDown()));
//        } else {
//            Actions.runBlocking(splineToPickThirdSpecimenFromObservationZone);
//
//            //sequence3 = 3rd Specimen dropped
//            Actions.runBlocking(sequence3); //3nd Specimen dropped
//
//            //sequence4 = Robot Parked
////            Actions.runBlocking(sequence4);
////            telemetry.addData("Forth Sequence:", runtime.seconds());
//        }
//
//        telemetry.update();
//    }
//
//}
//
//
//
