package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo {
    private Servo extendo = null;
    private Servo elbow = null;
    private Servo roller = null;

    public Extendo(HardwareMap hardwareMap) {
        extendo = hardwareMap.get(Servo.class, "extendo"); // chub 0
        roller = hardwareMap.get(Servo.class, "roller"); // chub 1
        elbow = hardwareMap.get(Servo.class, "elbow"); // chub 5
    }

    public void initialize() {
        extendo.setPosition(XBot.EXTENDO_MIN);
        elbow.setPosition(XBot.ELBOW_VERTICAL + 0.05);
    }

    public Action extend() {
        return new Action () {
            private boolean initialized = false;

            //Run called repeatedly unless return false
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    roller.setPosition(XBot.ROLLER_GRAB_SAMPLE);
                    elbow.setPosition(XBot.ELBOW_MAX);
                    extendo.setPosition(XBot.EXTENDO_MAX);
                    initialized = true;
                }
                double pos = extendo.getPosition();
                packet.put("extendo position", pos);
                return Math.abs(pos - XBot.EXTENDO_MAX) > 0.1;
            }
        };
    }

    public Action elbowVertical() {
        return new Action () {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    roller.setPosition(XBot.ROLLER_STOP);
                    elbow.setPosition(XBot.ELBOW_VERTICAL);
                    extendo.setPosition(XBot.EXTENDO_MIN);
                    initialized = true;
                }
                double pos = elbow.getPosition();
                packet.put("extendo position", pos);
                return Math.abs(pos - XBot.ELBOW_VERTICAL) > 0.05;
            }
        };
    }

    public Action elbowMin() {
        return new Action () {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    roller.setPosition(XBot.ROLLER_STOP);
                    elbow.setPosition(XBot.ELBOW_MIN);
                    extendo.setPosition(XBot.EXTENDO_MIN);
                    initialized = true;
                }
                double pos = elbow.getPosition();
                packet.put("Elbow position", pos);
                return Math.abs(pos - XBot.ELBOW_MIN) > 0.02;
            }
        };
    }
}
