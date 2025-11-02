package org.firstinspires.ftc.teamcode.stellarstructure.runnables;

import org.firstinspires.ftc.teamcode.stellarstructure.Subsystem;

public class DefaultDirective extends Directive {
    public DefaultDirective(Subsystem subsystem) {
        setRequiredSubsystems(subsystem);
        setInterruptible(true);
    }

    @Override
    public void start(boolean hadToInterruptToStart) {}

    @Override
    public void update() {}

    @Override
    public void stop(boolean interrupted) {}

    @Override
    public final boolean isFinished() {
        return false;
    }
}
