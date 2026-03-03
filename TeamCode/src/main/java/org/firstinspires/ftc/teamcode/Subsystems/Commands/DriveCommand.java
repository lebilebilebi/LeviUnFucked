package org.firstinspires.ftc.teamcode.Subsystems.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.Point;

public class DriveCommand {
    public static class StickInputs extends CommandBase {
        private final Drive drive;
        private final GamepadEx gamepadEx;

        public StickInputs(Drive drive, GamepadEx gamepadEx) {
            addRequirements(drive);
            this.drive = drive;
            this.gamepadEx = gamepadEx;
        }
        @Override
        public void execute(){
            drive.stickInput(gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX());
        }
    }
    public static class DriveToPoint extends CommandBase {
        private final Drive drive;
        private final Point point;


        public DriveToPoint(Drive drive, Point point) {
            addRequirements(drive);
            this.point = point;
            this.drive = drive;
        }
        @Override
        public void initialize() {
            drive.trajectoryStartSequence();
            drive.setTargetPoint(point);
        }

        @Override
        public boolean isFinished() {
            return drive.isAtTarget();
        }
    }
    public static class TurnToHeading extends CommandBase {
        private final Drive drive;
        private final double heading;


        public TurnToHeading(Drive drive, double heading) {
            addRequirements(drive);
            this.heading = heading;
            this.drive = drive;
        }
        @Override
        public void initialize() {
            drive.trajectoryStartSequence();
            drive.setTargetHeading(heading);
        }

        @Override
        public boolean isFinished() {
            return drive.isAtTarget();
        }
    }
}
