package org.firstinspires.ftc.teamcode;

public class XBot {
    static final double     EXTENDO_MIN = 0.0; // Extendo in starging position
    static final double     EXTENDO_HALF = 0.5; // Extendo in half extended position
    static final double     EXTENDO_MAX = 1.0; // Extendo in fully extended position

    static final double     ELBOW_MIN = 0.0; // Elbow folded back
    static final double     ELBOW_STRAIGHT = 0.8; // Elbow up front - but straight
    static final double     ELBOW_MAX = 1.0; // Elbow up front - but dropped down to pick sample
    static final double     ELBOW_TARGET_POSITION = 0.3; // Specific position for the elbow servo

    static final double     CLAW_CLOSE = 0.0; // Close position, Specimen in hand
    static final double     CLAW_OPEN = 0.2; // Position when we pick Specimen or open claw to drop
    static final double     CLAW_FULLY_OPEN = 1.0; // Maximum open position - may be used when dropping Sample to bucket

    static final double     ROLLER_FORWARD = 0.0; // Roller spinning forward - picking sample
    static final double     ROLLER_BACKWARD = 0.0; // Roller Spinning backward - releasing sample

    static final double     SERVO_MIN = 0.0; // Minimum servo position
    static final double     SERVO_MAX = 1.0; // Maximum servo position

    // Constants for servo position control
    static final double     SERVO_INCREMENT = 0.05; // Increment size for position changes

    // eg: Motor Encoder 117 = 384.5, 223 = 751.8, 312 = 537.7, 435 = 1425.1
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;// 384.5; // 1425.1;
    static final double     PULLEY_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV) / (PULLEY_DIAMETER_INCHES * Math.PI); //81.6 ticks per inch
    static final double     VIPER_DRIVE_SPEED = 1.0;

    static final double     VIPER_HOME = 0.0; // Viper zero position
    static final double     VIPER_PICK_SPECIMEN = 0.0; // Viper zero position
    static final double     VIPER_DROP_SPECIMEN = 0.0; // Viper zero position
    static final double     VIPER_DROP_SAMPLE_LOWER_BUCKET = 0.0; // Viper zero position
    static final double     VIPER_DROP_SAMPLE_HIGHER_BUCKET = 0.0; // Higher Bucket, also max position
    static final double     VIPER_MAX = VIPER_DROP_SAMPLE_HIGHER_BUCKET; // Viper max position


}
