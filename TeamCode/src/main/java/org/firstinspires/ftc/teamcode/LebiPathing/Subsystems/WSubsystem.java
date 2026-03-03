package org.firstinspires.ftc.teamcode.LebiPathing.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

public abstract class WSubsystem extends SubsystemBase {
    abstract public void read();
    abstract public void loop();
    abstract public void write();
}
