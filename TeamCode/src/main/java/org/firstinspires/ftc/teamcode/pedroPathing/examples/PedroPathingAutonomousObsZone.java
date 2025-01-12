package org.firstinspires.ftc.teamcode.pedroPathing.examples;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.OuttakeSlidesPosition;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "PedroPathingAutonomous", group = "AutonomousOpModes")
public class PedroPathingAutonomousObsZone extends OpMode{
    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private Path park;
    private PathChain scorePreLoad,goToFirstSamplePosition, pushFirstSampleIntoObsZone,
            goToSecondSamplePosition,pushSecondSampleIntoObsZone,
            goToThirdSamplePosition, pushThirdSampleIntoObsZone,
            gotoSpecimenFromTheWall, scoreSecondSpecimen, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;


    private final Pose startPose = new Pose(8, 65, Math.toRadians(0));  // Starting position
    private final Pose scorePreLoadSpecimenPose = new Pose(36, 121, Math.toRadians(0)); // Scoring position
    private final Pose goToFirstSamplePose = new Pose(58.043, 22.979, Math.toRadians(180)); // Scoring position
    private final Pose goToFirstSampleObsZonePose = new Pose(12.000, 23.000, Math.toRadians(180));
    private final Pose goToSecondSamplePose = new Pose(57.000, 15.000, Math.toRadians(180));
    private final Pose goToSecondSampleObsZonePose = new Pose(12.000, 15.000, Math.toRadians(180));
    private final Pose goToThirdSamplePose = new Pose(57.000, 7.000, Math.toRadians(180));

    private final Pose scorePreload = new Pose (57.000, 7.000, Math.toRadians(180));

    private final Pose scoreSpecimenPose = new Pose(57.000, 7.000, Math.toRadians(180));
    private final Pose goToThirdSampleObsZonePose = new Pose(12.000, 8.000, Math.toRadians(180));

    private final Pose grabSpecimanFromTheWall = new Pose();
    private final Pose scoreFirstSpecimenPose = new Pose(58.043, 22.979, Math.toRadians(315)); // Scoring position
    private final Pose scoreSecondSpecimenPose = new Pose(36, 121, Math.toRadians(315)); // Scoring position
    private final Pose scoreThirdSpecimenPose = new Pose(36, 121, Math.toRadians(315)); // Scoring position
    private final Pose scoreFourthSpecimenPose = new Pose(36, 121, Math.toRadians(315)); // Scoring position

    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // First sample pickup
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Second sample pickup
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Third sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90)); // Control point for curved path

    private Outtake outtake = null;

    public void buildPaths() {
        // Path for scoring preload
        scorePreLoad = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Point(startPose),
                    new Point(scorePreLoadSpecimenPose)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreLoadSpecimenPose.getHeading())
                .build();


        // Path chains for picking up and scoring samples
        goToFirstSamplePosition = follower.pathBuilder()

                .addPath(new BezierCurve(
                        new Point(40.000, 65.000, Point.CARTESIAN),
                        new Point(26.553, 14.468, Point.CARTESIAN),
                        new Point(62.979, 50.894, Point.CARTESIAN),
                        new Point(58.043, 22.979, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(scorePreLoadSpecimenPose.getHeading(), goToFirstSamplePose.getHeading())
                .setReversed(true)
                .build();

        pushFirstSampleIntoObsZone = follower.pathBuilder()

                .addPath(
                        new BezierLine(
                                new Point(58.043, 22.979, Point.CARTESIAN),
                                new Point(12.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        goToSecondSamplePosition = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(12.000, 23.000, Point.CARTESIAN),
                                new Point(59.915, 25.362, Point.CARTESIAN),
                                new Point(57.000, 15.000, Point.CARTESIAN)
                        )
                )
              .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pushSecondSampleIntoObsZone = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(57.000, 15.000, Point.CARTESIAN),
                                new Point(12.000, 15.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        goToThirdSamplePosition = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(12.000, 15.000, Point.CARTESIAN),
                                new Point(61.447, 17.702, Point.CARTESIAN),
                                new Point(57.000, 7.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pushThirdSampleIntoObsZone = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(57.000, 7.000, Point.CARTESIAN),
                                new Point(12.000, 8.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        gotoSpecimenFromTheWall = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(12.000, 8.000, Point.CARTESIAN),
                                new Point(9.000, 32.000, Point.CARTESIAN)
                        )
                )
                .build();
        scoreSecondSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goToThirdSampleObsZonePose), new Point(grabSpecimanFromTheWall)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scoreSpecimenPose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpecimenPose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scoreSpecimenPose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scoreSpecimenPose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scoreSpecimenPose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpecimenPose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scoreSpecimenPose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scoreSpecimenPose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scoreSpecimenPose.getHeading())
                .build();

        // Curved path for parking
        park = new Path(new BezierCurve(new Point(scoreSpecimenPose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scoreSpecimenPose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(scorePickup1);
                outtake.setOuttakeSlidesPosition(OuttakeSlidesPosition.HOOK_SPECIMEN_TOP_RUNG);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (follower.getPose().getX() > (scoreSpecimenPose.getX() - 1) && follower.getPose().getY() > (scoreSpecimenPose.getY() - 1)) {
                    follower.followPath(goToFirstSamplePosition, true);
                    setPathState(2);
                }
                break;

            case 2: // Wait until the robot is near the first sample pickup position
                if (follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (scoreSpecimenPose.getX() - 1) && follower.getPose().getY() > (scoreSpecimenPose.getY() - 1)) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            case 4: // Wait until the robot is near the second sample pickup position
                if (follower.getPose().getX() > (pickup2Pose.getX() - 1) && follower.getPose().getY() > (pickup2Pose.getY() - 1)) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (scoreSpecimenPose.getX() - 1) && follower.getPose().getY() > (scoreSpecimenPose.getY() - 1)) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            case 6: // Wait until the robot is near the third sample pickup position
                if (follower.getPose().getX() > (pickup3Pose.getX() - 1) && follower.getPose().getY() > (pickup3Pose.getY() - 1)) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            case 7: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (scoreSpecimenPose.getX() - 1) && follower.getPose().getY() > (scoreSpecimenPose.getY() - 1)) {
                    follower.followPath(park, true);
                    setPathState(8);
                }
                break;

            case 8: // Wait until the robot is near the parking position
                if (follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
                    setPathState(-1); // End the autonomous routine
                }
                break;
        }
    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This method is called once at the init of the OpMode. **/

    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


}
