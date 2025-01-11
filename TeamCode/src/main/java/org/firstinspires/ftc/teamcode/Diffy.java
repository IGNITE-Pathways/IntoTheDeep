package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


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
        setDiffyPosition(DiffyPosition.TRANSFER_SAMPLE);
        setIntakeClawPosition(ClawPosition.OPEN);
    }

    public void setDiffyPosition(DiffyPosition position) {
        this.savedPosition = position;
        setDiffyPositions(position.getLeftPos(), position.getRightPos());
    }

    public void setDiffyPositions(double leftPos, double rightPos) {
        diffyLeft.setPosition(leftPos);
        diffyRight.setPosition(rightPos);
    }

    public void flipDiffyVerticalPosition() {
        switch (savedPosition) {
            case FLAT:
                setDiffyPosition(DiffyPosition.DOWN_PARALLEL);
                break;
            case DOWN_PARALLEL:
                setDiffyPosition(DiffyPosition.FLAT);
                break;
            default:
                setDiffyPosition(DiffyPosition.FLAT);
                break;
        }
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

        SampleColor detectedColor = SampleColor.UNKNOWN;
        if (redValue > 100 && greenValue > 100 && blueValue < 100) {
            detectedColor = SampleColor.YELLOW;
        } else if (blueValue > redValue && blueValue > greenValue) {
            detectedColor = SampleColor.BLUE;
        } else if (redValue > blueValue && redValue > greenValue) {
            detectedColor = SampleColor.RED;
        }

        if (intakeSensor instanceof com.qualcomm.robotcore.hardware.SwitchableLight) {
            // Cast, then enable or disable the LED
            ((com.qualcomm.robotcore.hardware.SwitchableLight) intakeSensor).enableLight(false);  // turn on
        }
        return detectedColor;
    }

    public boolean isClawClosed() {
        return intakeClaw.getPosition() < 0.4;
    }

    public boolean isClawOpen() {
        return intakeClaw.getPosition() > 0.9;
    }

    public void closeClaw() {
        intakeClaw.setPosition(0.5);
    }

    public void openClaw() {
        intakeClaw.setPosition(1);
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

}