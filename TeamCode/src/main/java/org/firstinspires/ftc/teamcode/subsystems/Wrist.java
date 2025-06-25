package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist extends SubsystemBase {
    private DcMotor wrist = null;
    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_SAMPLE = 270;
    private static final int WRIST_POSITION_SPEC = 0;



    private enum RobotState{
        INIT
    }

    private Wrist.RobotState currentState = Wrist.RobotState.INIT;

    private int targetWrist = 0;

    public Wrist(final HardwareMap hardwareMap) {
        wrist= hardwareMap.get(DcMotor.class,"wrist");
        wrist.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}
