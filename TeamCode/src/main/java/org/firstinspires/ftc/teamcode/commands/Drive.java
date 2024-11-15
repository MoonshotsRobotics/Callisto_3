package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.RoadRunner;
import org.firstinspires.ftc.teamcode.util.experiments.PureMecanum;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Drive extends CommandBase {
    private final Mecanum mecanum;
    private GamepadEx player1;
    private Callisto robot;

    private final FtcDashboard dashboard;

    private double strafeSpeed;
    private double forwardSpeed;
    private double turnSpeed;

    public Drive(Callisto robot) {
        // pulling off some handy references
        this.robot = robot;
        this.mecanum = robot.mecanum;
        player1 = robot.player1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robot.mecanum);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void initialize(){
        // Do you need to set up anything when the command gets first added to the scheduler?
    }

    @Override
    public void execute() {

        // Slow mode activated if right bumper or lift is up
        double speedMod = robot.opMode.gamepad1.right_bumper || robot.lift.isUp()
                ? 0.25 : 1; // slow mode

        // INTERPRET CONTROLLER
        double forward = applyDeadZone(player1.getLeftY());
        double strafe = applyDeadZone(player1.getLeftX());
        double turn = applyDeadZone(-player1.getRightX());

        robot.telemetry.addData("Right Stick", "X: %.2f Y: %.2f", player1.getRightX(), player1.getRightY());
        robot.telemetry.addData("perp", mecanum.rightFront.getCurrentPosition());

        // Drive the robot with adjusted inputs:
        mecanum.drive(forward * speedMod, strafe * speedMod, turn * speedMod);
    }

    public boolean isFinished() {
        // currently just a placeholder... the default 
        return false;
    }

    /**
     * Helper function for applying dead zone
     */
    private double applyDeadZone(double input) {
        return Math.abs(input) <= Constants.INPUT_THRESHOLD ? 0.0d : input;
    }
}
