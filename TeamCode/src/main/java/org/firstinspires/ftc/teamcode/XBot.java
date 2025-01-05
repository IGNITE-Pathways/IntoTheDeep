package org.firstinspires.ftc.teamcode;

public class XBot {
    static final double     EXTENDO_MIN = 1.0; // Extendo in starging position
    static final double     EXTENDO_HALF = 0.75; // Extendo in half extended position
    static final double     EXTENDO_MAX = 0.5; // Extendo in fully extended position

    static final double     ELBOW_MIN = 0.94; // Elbow folded back
    static final double     ELBOW_STRAIGHT = 0.1; // Elbow up front - but straight
    static final double     ELBOW_MAX = 0.02; // Elbow up front - but dropped down to pick sample
    static final double     ELBOW_VERTICAL = 0.42; // Elbow up front - but dropped down to pick sample

    static final double     CLAW_CLOSE = 0.45; // Close position, Specimen in hand
    static final double     CLAW_OPEN = 0.78; // Position when we pick Specimen or open claw to drop
    static final double     CLAW_FULLY_OPEN = 1.0; // Maximum open position - may be used when dropping Sample to bucket

    static final double     ROLLER_DUMP_SAMPLE = 1.0; // Roller spinning forward - picking sample
    static final double     ROLLER_GRAB_SAMPLE = 0.0; // Roller Spinning backward - releasing sample
    static final double     ROLLER_STOP = 0.5; // Roller Stopped Spinning

    static final double     SERVO_MIN = 0.0; // Minimum servo position
    static final double     SERVO_MAX = 1.0; // Maximum servo position

    // Constants for servo position control
    static final double     SERVO_INCREMENT = 0.03; // Increment size for position changes

    // eg: Motor Encoder 117 = 384.5, 223 = 751.8, 312 = 537.7, 435 = 1425.1
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;// 384.5; // 1425.1;
    static final double     PULLEY_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV) / (PULLEY_DIAMETER_INCHES * Math.PI); //81.6 ticks per inch
    static final double     VIPER_DRIVE_SPEED = 1.0;

    //All values in Inches
    static final double     VIPER_HOME = 0.0; // Viper zero position
    static final double     VIPER_PICK_SPECIMEN = 3.0; // Viper zero position 521
    static final double     VIPER_DROP_SPECIMEN = 18.5; // Viper zero position 2064
    static final double     DROPPED_SPECIMEN = VIPER_DROP_SPECIMEN - 3.5; // Viper position to specimen on bar

    static final double     VIPER_DROP_SAMPLE_LOWER_BUCKET = 19.5; // Viper zero position 2223
    static final double     VIPER_DROP_SAMPLE_HIGHER_BUCKET = 37.3; // Higher Bucket, also max position 4138
    static final double     VIPER_MAX = VIPER_DROP_SAMPLE_HIGHER_BUCKET; // Viper max position
    static final double     VIPER_LEVEL1ASCENT = VIPER_DROP_SPECIMEN - 11;

    static final double     DIFFY_PICK_ANGLE = 0;
    static final double     DIFFY_TRANFER_ANGLE = 0;

    //@TODO: Make sure to change the value of DIFFY_PICK_ANGLE
}
