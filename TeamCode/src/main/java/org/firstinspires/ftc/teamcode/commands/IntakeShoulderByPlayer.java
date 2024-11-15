package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


public class IntakeShoulderByPlayer extends CommandBase {

    private final Callisto robot;
    private final Intake intake;

    public IntakeShoulderByPlayer (Callisto robot){
        this.robot = robot;
        this.intake = this.robot.intake;

        addRequirements(robot.intakeExperimental);
    }

    @Override
    public void execute() {
        double power = robot.player2.getRightY();
        robot.telemetry.addData("is Command Running", true);

        // Deadzone
        if(Math.abs(power) < 0.05){
            power = 0;
        }
        intake.shoulderMotor.set(power);
    }

    public boolean isFinished() {
        return false;
    }
}
