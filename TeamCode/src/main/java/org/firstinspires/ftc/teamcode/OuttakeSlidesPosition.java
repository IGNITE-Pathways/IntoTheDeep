package org.firstinspires.ftc.teamcode;

public enum OuttakeSlidesPosition {
    CLOSE ("CLOSE", 0.0), //0 inches
    PRE_TRANSFER ("PRE_TRANSFER", 3.35), //2 inches
    TRANSFER ("TRANSFER", 2.4), //2 inches
    DROP_SAMPLE ("DROP_SAMPLE", 28.0), //28 inches
    HOOK_SPECIMEN_TOP_RUNG("HOOK_SPECIMEN_TOP_RUNG", 13),
    POST_HOOK("POST_HOOK", 7); //14 inches

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