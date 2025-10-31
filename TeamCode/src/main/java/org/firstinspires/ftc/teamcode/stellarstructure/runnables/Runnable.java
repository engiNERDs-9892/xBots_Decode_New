package org.firstinspires.ftc.teamcode.stellarstructure.runnables;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.stellarstructure.Scheduler;
import org.firstinspires.ftc.teamcode.stellarstructure.Subsystem;
import org.firstinspires.ftc.teamcode.stellarstructure.Trigger;
import org.firstinspires.ftc.teamcode.stellarstructure.conditions.Condition;

import java.util.ArrayList;
import java.util.List;

public abstract class Runnable {
    //no required subsystems by default
    private Subsystem[] requiredSubsystems = {};
    private Condition[] startingConditions = {};
    private final List<Trigger> ownedTriggers = new ArrayList<>();

    //interruptible by default
    private boolean interruptible = true;

    private boolean hasFinished = false;

    public abstract void start(boolean hadToInterruptToStart);

    public abstract void update();

    public abstract void stop(boolean interrupted);

    public abstract boolean isFinished();
    public final void addTrigger(Trigger trigger) {
        ownedTriggers.add(trigger);
    }

    public final void removeTrigger(Trigger trigger) {
        ownedTriggers.remove(trigger);
    }

    public final List<Trigger> getOwnedTriggers() {
        return this.ownedTriggers;
    }

    public final void setRequires(@NonNull Subsystem... subsystems) {
        requiredSubsystems = subsystems;
    }

    public final Subsystem[] getRequiredSubsystems() {
        return requiredSubsystems;
    }

    public final void setInterruptible(boolean interruptible) {
        this.interruptible = interruptible;
    }

    public final Condition[] getStartingConditions() {
        return startingConditions;
    }

    public final void setStartingConditions(Condition... startingConditions) {
        this.startingConditions = startingConditions;
    }

    public final boolean getInterruptible() {
        return interruptible;
    }

    public final boolean getHasFinished() {
        return hasFinished;
    }

    public final void setHasFinished(boolean finished) {
        hasFinished = finished;
    }

    public final void schedule() {
        Scheduler.getInstance().schedule(this);
    }
}
