package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExperimental;

import java.util.Timer;

public class IntakeShoulderUpExperimental extends CommandBase {
    private Callisto m_robot;
    private IntakeExperimental m_intake;

    private Timing.Timer timer;
    private long TIMEOUT = 12;

    private int targetPosition;

    public IntakeShoulderUpExperimental(Callisto robot, int targetPosition) {
        m_robot = robot;
        m_intake = m_robot.intakeExperimental;

        this.targetPosition = targetPosition;

        timer = new Timing.Timer(TIMEOUT);

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        m_intake.shoulderMotor.setPower(0.5);
    }

    @Override
    public boolean isFinished() {
        return (m_intake.shoulderMotor.getCurrentPosition() >= (targetPosition - 5)) || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.shoulderMotor.setPower(0);
    }
}
