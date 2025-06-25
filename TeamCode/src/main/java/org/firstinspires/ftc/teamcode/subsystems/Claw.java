package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Claw extends SubsystemBase{
    private Servo claw = null;
    public static final double ClawServo_CLOSE = 0.68;
    public static final double ClawServo_OPEN = 0.5;
    private boolean isClawOpen;


    public Claw(HardwareDevice hardwareMap) {
        claw.getClass();
        claw.getDeviceName();
        claw.setPosition(ClawServo_CLOSE);
    }
    public void switchClaw() {
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
    public Command openClawCommand() {
        return new InstantCommand(this::openClaw);
    }

    public Command closeClawCommand() {
        return new InstantCommand(this::closeClaw);
    }

    public void setServoController(boolean enable) {
        if (enable) {
            claw.getController().pwmEnable();

        } else {
            claw.getController().pwmDisable();

        }
    }


}
