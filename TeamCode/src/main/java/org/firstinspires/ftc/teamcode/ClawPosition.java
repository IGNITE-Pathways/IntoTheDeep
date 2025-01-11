package org.firstinspires.ftc.teamcode;

public enum ClawPosition {
    OPEN("OPEN", 0.0),
    CLOSE("CLOSE", 1.0);

    private final String name;
    private final double clawPos;

    ClawPosition(String name, double clawPos) {
        this.name = name;
        this.clawPos = clawPos;
    }

    public String getName() {
        return name;
    }

    public double getClawPos() {
        return clawPos;
    }
}