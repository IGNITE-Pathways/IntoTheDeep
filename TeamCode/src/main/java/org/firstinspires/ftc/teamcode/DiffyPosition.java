package org.firstinspires.ftc.teamcode;

public enum DiffyPosition {
    TRANSFER_SAMPLE("TRANSFER_SAMPLE", 0.76, 0.71), //0.74, 0.69, 0.8, 0.75
    TRANSFER_SPECIMEN("TRANSFER_SPECIMEN", 0.49, .99),
    FLAT("FLAT", 0.40, 0.34),
    DOWN_PARALLEL("DOWN_PARALLEL", 0.12, 0.06), //0.10, 0.03
    DOWN_CLOCKWISE("DOWN_CLOCKWISE", 0.04, 0.12),
    DOWN_ANTI_CLOCKWISE("DOWN_ANTI_CLOCKWISE", 0.15, 0);

    private final String name;
    private final double leftPos;
    private final double rightPos;

    DiffyPosition(String name, double leftPos, double rightPos) {
        this.name = name;
        this.leftPos = leftPos;
        this.rightPos = rightPos;
    }

    public String getName() {
        return name;
    }

    public double getLeftPos() {
        return leftPos;
    }

    public double getRightPos() {
        return rightPos;
    }
}