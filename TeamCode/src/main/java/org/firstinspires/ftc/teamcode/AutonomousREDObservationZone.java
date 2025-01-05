//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous(name = "Auto RED Obs Zone", group = "Linear OpMode")
//@Disabled
//public class AutonomousREDObservationZone extends LinearOpMode {
//
//    // Declare OpMode members for each of the 4 motors.
//    private final ElapsedTime runtime = new ElapsedTime();
//    boolean rolling = false;
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
//
//    private DigitalChannel redLED;
//    private DigitalChannel greenLED;
//
//    @Override
//    public void runOpMode() {
//
//        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "rightback"); //rightback
//        leftBackDrive = hardwareMap.get(DcMotorEx.class, "rightfront"); //rightfront`
//        rightBackDrive = hardwareMap.get(DcMotorEx.class, "leftfront"); //leftfront
//        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "leftback"); //leftback
//
//        redLED = hardwareMap.get(DigitalChannel.class, "redled");//7
//        greenLED = hardwareMap.get(DigitalChannel.class, "greenled");//6
//
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//        // change LED mode from input to output
//        redLED.setMode(DigitalChannel.Mode.OUTPUT);
//        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
//
//        Pose2d beginPose = new Pose2d(8, -62, Math.toRadians(90));
//        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
//        Viper viper = new Viper(hardwareMap);
//        Extendo extendo = new Extendo(hardwareMap);
//        viper.initialize();
//        extendo.initialize();
//
//        waitForStart();
//        runtime.reset();
//
//        Action moveToDropSpecimenLocation = drive.actionBuilder(new Pose2d(8, -62, Math.toRadians(90)))
//                .strafeTo(new Vector2d(2, -36))
//                .build();
//
//        //Move to drop specimen while rising
//        SequentialAction raiseViperWhenMovingToDropSpecimen = new SequentialAction(
//                viper.getReadyToDropSpecimen(),
//                moveToDropSpecimenLocation);
//
//        //drops preloaded specimen on the chamber
//        SequentialAction dropSpecimenSequence = new SequentialAction(
//                raiseViperWhenMovingToDropSpecimen,
//                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
//                viper.openClaw());
//
//        Action moveToThePositionOfFIRSTSample =
//                drive.actionBuilder(new Pose2d(2, -36, Math.toRadians(90)))
//                        .strafeTo(new Vector2d(33, -40)) // drives to the right
//                        .strafeTo((new Vector2d(35, -35))) // gets ready to do a nice spline, without hitting the truss holding up the submersible
//                        .splineToLinearHeading(new Pose2d(46, -10, Math.toRadians(-90)), Math.toRadians(270)) // splines to the first sample
//                        .build();
//
//        //Move the viper slide down while moving to the position to pick next sample
//        SequentialAction moveViperDownWhenMovingToTheFIRSTSample = new SequentialAction(
//                dropSpecimenSequence,
//                viper.driveToPositionInInches(XBot.VIPER_HOME),
//                moveToThePositionOfFIRSTSample);
//
//        Action pushTheSampleIntoTheObservationZone =
//                drive.actionBuilder(new Pose2d(46, -10, Math.toRadians(-90)))
//                        .strafeTo(new Vector2d(46, -60)) // pushes first sample into observation zone
//                        .build();
//
//        SequentialAction SequenceOfActions = new SequentialAction(
//                dropSpecimenSequence,
//                moveViperDownWhenMovingToTheFIRSTSample,
//                pushTheSampleIntoTheObservationZone);
//
//        Actions.runBlocking(SequenceOfActions);
//        telemetry.addData("Time Used", runtime.seconds());
//
////        Action pushSecondSampleToObsZone = drive.actionBuilder(new Pose2d(46, -55, Math.toRadians(360)))
////                .setReversed(true)
////                .splineToConstantHeading(new Vector2d(55, -16), Math.toRadians(360))  // goes to second sample
////                .setReversed(false)
////                .strafeTo(new Vector2d(55, -55)) // pushes 2nd sample into the observation zone
////                .build();
//
////        Action pickSecondSpecimen = drive.actionBuilder(new Pose2d(46, -60, Math.toRadians(-90)))
////                .strafeTo(new Vector2d(62.5, -57)) // goes to hook specimen with the CLAW
////                .build();
//
////        Actions.runBlocking(pushSecondSampleToObsZone);
//
//        SequentialAction pickAction = new SequentialAction(
//                viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN),
//                new SleepAction(.2),
//                viper.closeClaw(),
//                new SleepAction(.2),
//                viper.getReadyToDropSpecimen(),
//                new SleepAction(.2));
//
//        Action moveToDropSecondSpecimen = drive.actionBuilder(new Pose2d(46, -60, Math.toRadians(-90)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(12, -32, Math.toRadians(90)), Math.toRadians(90)) // splines to chamber to hook second specimen
//                .build();
//
//        Actions.runBlocking(new SequentialAction(pickAction, moveToDropSecondSpecimen,
//                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
//                viper.openClaw()));
//
//        Action parkAction = drive.actionBuilder(new Pose2d(12, -32, Math.toRadians(90)))
//                                .strafeTo(new Vector2d(-44,62)).build();
//
//        Actions.runBlocking(new SequentialAction(parkAction, extendo.elbowVertical(),
//                viper.driveToPositionInInches(XBot.VIPER_HOME)));
//
//        // Telemetry
//        telemetry.addData("Time Used", runtime.seconds());
//
//        // Show the elapsed game time and wheel power.
//        telemetry.update();
//    }
//
//}
//
//
//
