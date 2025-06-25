package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class Arm extends SubsystemBase {
    private DcMotor arm = null;

    private static final int ARM_POSITION_INIT = 100;
    private static final int ARM_POSITION_INTAKE = 475;
    private static final int ARM_POSITION_WALL_GRAB = 4300;
    private static final int ARM_POSITION_WALL_UNHOOK = 1200;
    private static final int ARM_POSITION_HOVER_HIGH = 1550;
    private static final int ARM_POSITION_CLIP_HIGH = 1950;


    private enum RobotState{
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
    }
    private RobotState currentState = RobotState.INIT;

    private int targetArm = 0;

    public Arm(final HardwareDevice hardwareMap) {
        arm.getClass();
        arm.getDeviceName();
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}
