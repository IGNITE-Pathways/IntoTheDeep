package org.firstinspires.ftc.teamcode;

public enum OuttakeArmPosition {

    TRANSFER ("TRANSFER", 250), //0 Degrees
    SAMPLE_DROP("SAMPLE_DROP", 20), //255 Degrees
    SPECIMEN_DROP("SPECIMEN_DROP", 75), //210 Degrees,
    POST_SPECIMEN_DROP("POST_SPECIMEN_DROP", 95), //190 Degrees,
    FACING_DOWN("FACING_DOWN", 200); //70 Degrees

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
