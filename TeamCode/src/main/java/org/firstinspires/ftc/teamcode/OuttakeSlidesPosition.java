package org.firstinspires.ftc.teamcode;

public enum OuttakeSlidesPosition {
    CLOSE ("CLOSE", 0.0), //0 inches
    TRANSFER ("TRANSFER", 2.15), //2 inches
    DROP_SAMPLE ("DROP_SAMPLE", 28.0), //28 inches
    HOOK_SPECIMEN_TOP_RUNG("HOOK_SPECIMEN_TOP_RUNG", 14.0); //14 inches

    private String name;
    private double position;

    OuttakeSlidesPosition(String name, double position) {
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