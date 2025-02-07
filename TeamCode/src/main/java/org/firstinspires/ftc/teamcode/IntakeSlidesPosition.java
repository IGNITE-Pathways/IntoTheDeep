package org.firstinspires.ftc.teamcode;

public enum IntakeSlidesPosition {
    CLOSE ("CLOSE", 0),
    SHORT ("SHORT", 3),
    TRANSFER ("TRANSFER", .5),
    FULL ("FULL", 9);

    private String name;
    private double position;

    IntakeSlidesPosition(String name, double position) {
        this.name = name;
        this.position = position;
    }

    public String getName() {
        return name;
    }

    public double getPosition() {
        return position;
    }
}
