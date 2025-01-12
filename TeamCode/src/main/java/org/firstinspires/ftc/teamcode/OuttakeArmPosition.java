package org.firstinspires.ftc.teamcode;

public enum OuttakeArmPosition {

    TRANSFER ("TRANSFER", 0), //0 Degrees
    SAMPLE_DROP("SAMPLE_DROP", Outtake.MAX_ROTATION_DEGREES), //255 Degrees
    SPECIMEN_DROP("SPECIMEN_DROP", 210), //210 Degrees,
    POST_SPECIMEN_DROP("POST_SPECIMEN_DROP", 190), //190 Degrees,
    FACING_DOWN("FACING_DOWN", 70); //70 Degrees

    String name;
    double degrees;

    OuttakeArmPosition(String name, double degrees) {
        this.name = name;
        this.degrees = degrees;
    }

    public String getName() {
        return name;
    }

    public double getDegrees() {
        return degrees;
    }
}
