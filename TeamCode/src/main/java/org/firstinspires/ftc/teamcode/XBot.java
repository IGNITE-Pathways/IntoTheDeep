package org.firstinspires.ftc.teamcode;

public class XBot {
    static final double     EXTENDO_MAX = 0.5; // Extendo in fully extended position

    static final double     ELBOW_STRAIGHT = 0.1; // Elbow up front - but straight

    static final double     CLAW_CLOSE = 0.45; // Close position, Specimen in hand
    static final double     CLAW_OPEN = 0.78; // Position when we pick Specimen or open claw to drop

    // Constants for servo position control
    static final double     SERVO_INCREMENT = 0.03; // Increment size for position changes

    // eg: Motor Encoder 117 = 384.5, 223 = 751.8, 312 = 537.7, 435 = 1425.1
    static final double     COUNTS_PER_MOTOR_REV    = 1425.1;// 384.5; // 1425.1;
    static final double     PULLEY_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV) / (PULLEY_DIAMETER_INCHES * Math.PI); //81.6 ticks per inch
    static final double     VIPER_DRIVE_SPEED = 1.0;

    //All values in Inches
    static final double     VIPER_DROP_SPECIMEN = 18.5; // Viper zero position 2064

}
