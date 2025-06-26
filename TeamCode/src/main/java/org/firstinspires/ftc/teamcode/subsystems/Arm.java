package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.RobotState;

public class Arm extends SubsystemBase {
    private DcMotor arm = null;

    private static final int ARM_POSITION_INIT = 100;
    private static final int ARM_POSITION_INTAKE = 475;
    private static final int ARM_POSITION_WALL_GRAB = 4300;
    private static final int ARM_POSITION_WALL_UNHOOK = 4100;
    private static final int ARM_POSITION_HOVER_HIGH = 1550;
    private static final int ARM_POSITION_CLIP_HIGH = 1950;



    private RobotState currentState = RobotState.INIT;

    private int targetArm = 0;

    public Arm(DcMotor Arm) {
        this.arm = Arm;
        targetArm = ARM_POSITION_INIT;
    }

    public void wallgrab() {
        arm.setTargetPosition(ARM_POSITION_WALL_GRAB);

    }

    public void wallUnhook(){
        arm.setTargetPosition(ARM_POSITION_WALL_UNHOOK);

    };

    public void hover() {
        arm.setTargetPosition(ARM_POSITION_HOVER_HIGH);
    }

    public void clipScore() {
        arm.setTargetPosition(ARM_POSITION_CLIP_HIGH);
    }


}
