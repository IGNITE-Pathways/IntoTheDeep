package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Diffy {
    public Servo diffyLeft;
    public Servo diffyRight;
    public Servo intakeClaw = null;
    public ColorRangeSensor intakeSensor = null;
    private DiffyPosition savedPosition = null;

    private ClawPosition intakeClawPosition = ClawPosition.OPEN;

    public Diffy(HardwareMap hardwareMap) {
        diffyLeft = hardwareMap.get(Servo.class, "diffyleft");
        diffyRight = hardwareMap.get(Servo.class, "diffyright");
        diffyLeft.setDirection(Servo.Direction.REVERSE);
        intakeClaw = hardwareMap.get(Servo.class, "intakeclaw"); // ehub 4
        intakeSensor = hardwareMap.get(ColorRangeSensor.class, "intakesensor"); // ehub I2C 0
    }

    public void initialize() {
        setDiffyPosition(DiffyPosition.TRANSFER);
        setIntakeClawPosition(ClawPosition.OPEN);
        openClaw();
    }

    public void setDiffyPosition(DiffyPosition position) {
        this.savedPosition = position;
        setDiffyPositions(position.getLeftPos(), position.getRightPos());
    }

    public void setDiffyPositions(double leftPos, double rightPos) {
        diffyLeft.setPosition(leftPos);
        diffyRight.setPosition(rightPos);
    }

    public SampleColor getSampleColor() {
        if (intakeSensor instanceof com.qualcomm.robotcore.hardware.SwitchableLight) {
            // Cast, then enable or disable the LED
            ((com.qualcomm.robotcore.hardware.SwitchableLight) intakeSensor).enableLight(true);  // turn on
        }
        // Read raw RGB values
        int redValue   = intakeSensor.red();
        int greenValue = intakeSensor.green();
        int blueValue  = intakeSensor.blue();
        double distanceInMM = getDistance();
        if (distanceInMM > 100) return SampleColor.UNKNOWN;

        SampleColor detectedColor = SampleColor.UNKNOWN;
        if (redValue > 100 && greenValue > 100 && blueValue < 100) {
            detectedColor = SampleColor.YELLOW;
        } else if (blueValue > redValue && blueValue > greenValue && greenValue > redValue) {
            detectedColor = SampleColor.BLUE;
        } else if (redValue > blueValue && redValue > greenValue && blueValue > greenValue) {
            detectedColor = SampleColor.RED;
        } else if (redValue > greenValue && greenValue > blueValue){
            detectedColor = SampleColor.YELLOW;
        }

        if (intakeSensor instanceof com.qualcomm.robotcore.hardware.SwitchableLight) {
            // Cast, then enable or disable the LED
            ((com.qualcomm.robotcore.hardware.SwitchableLight) intakeSensor).enableLight(false);  // turn on
        }
        return detectedColor;
    }

    public boolean isClawOpen() {
        return intakeClaw.getPosition() < 0.25;
    }

    public boolean isClawClosed() {
        return intakeClaw.getPosition() > 0.95;
    }

    public void closeClaw() {
        intakeClaw.setPosition(ClawPosition.CLOSE.getClawPos());
    }

    public void openClaw() {
        intakeClaw.setPosition(ClawPosition.OPEN.getClawPos());
    }

    public void setIntakeClawPosition(ClawPosition position) {
        this.intakeClawPosition = position;
        switch (intakeClawPosition) {
            case CLOSE:
                if (!isClawClosed()) closeClaw();
                break;
            case OPEN:
                if (!isClawOpen()) openClaw();
                break;
        }
    }

    public double getClawPosition() {
        return intakeClaw.getPosition();
    }

    public void cycleDownPosition(int sign) {
        switch (savedPosition) {
            case DOWN_ANTI_CLOCKWISE:
                setDiffyPosition(DiffyPosition.DOWN_PARALLEL);
                break;
            case DOWN_PARALLEL:
                if (sign == 1) {
                    setDiffyPosition(DiffyPosition.DOWN_CLOCKWISE);
                } else {
                    setDiffyPosition(DiffyPosition.DOWN_ANTI_CLOCKWISE);
                }
                break;
            case DOWN_CLOCKWISE:
                setDiffyPosition(DiffyPosition.DOWN_PARALLEL);
                break;
        }
    }

    public void setClawPosition(double diffyPosition) {
        double dPos = Range.clip(diffyPosition, 0, 1);
        intakeClaw.setPosition(dPos);
    }

    public boolean isDiffyInPosition() {
        //compare savedPosition to diffyLeft and diffyRight
        return (Math.abs(savedPosition.getLeftPos() - diffyLeft.getPosition()) < 0.05) &&
                (Math.abs(savedPosition.getRightPos() - diffyRight.getPosition()) < 0.05);
    }

    public double getDistance() {
        return intakeSensor.getDistance(DistanceUnit.MM);
    }

    //This can misfire if claw is pointing downward
    public boolean isDiffyHoldingGameElement() {
//        if ((savedPosition == DiffyPosition.DOWN_CLOCKWISE) ||
//                (savedPosition == DiffyPosition.DOWN_PARALLEL) ||
//                (savedPosition == DiffyPosition.DOWN_ANTI_CLOCKWISE)) {
//
//        }
        return (isClawClosed() && (getDistance() < 110));
    }
}