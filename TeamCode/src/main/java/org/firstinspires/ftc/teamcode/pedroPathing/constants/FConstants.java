package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


// This acts as a method of updating FollowerConstants without direct access to it
public class FConstants { //This is how we change Follower Constants
    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.PINPOINT;

// hardware map names
        FollowerConstants.leftFrontMotorName = "leftfront";
        FollowerConstants.leftRearMotorName = "leftback";
        FollowerConstants.rightFrontMotorName = "rightfront";
        FollowerConstants.rightRearMotorName = "rightback";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        // We can change the value of any variable/constant of FollowerConstants
        FollowerConstants.mass = 11.33; // In kg

    }
}
