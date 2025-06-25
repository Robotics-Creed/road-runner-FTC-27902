package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Claw {
    private Servo claw = null;
    public static final double ClawServo_CLOSE = 0.5;
    public static final double ClawServo_OPEN = 0.68;
    boolean isClawOpen = false;

    public Claw(Servo Claw) {
        this.claw = Claw;
        claw.setPosition(ClawServo_CLOSE);
    }

    public void toggleClaw() {
        if (isClawOpen) {
            claw.setPosition(ClawServo_CLOSE);
        } else {
            claw.setPosition(ClawServo_OPEN);
        }
        isClawOpen = !isClawOpen;
    }

    public void openClaw() {
        isClawOpen = true;
        claw.setPosition(ClawServo_OPEN);
    }

    public void closeClaw() {
        isClawOpen = false;
        claw.setPosition(ClawServo_CLOSE);
    }

    public double getClawPosition() {
        return claw.getPosition();
    }

    public boolean getClawState() {
        return isClawOpen;
    }
}
