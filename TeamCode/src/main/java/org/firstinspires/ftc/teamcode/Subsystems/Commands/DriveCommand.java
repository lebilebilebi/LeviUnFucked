package org.firstinspires.ftc.teamcode.Subsystems.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;

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
}
