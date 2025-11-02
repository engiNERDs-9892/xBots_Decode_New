package org.firstinspires.ftc.teamcode.stellarstructure.runnables;

import androidx.annotation.NonNull;

public class Procedure extends Runnable {
    Directive[] directives;
    private int currentDirectiveIndex = 0;

    public Procedure(@NonNull Directive... directives) {
        if (directives.length == 0) {
            throw new IllegalArgumentException("No directives provided");
        }

        this.directives = directives;

        setRequiredSubsystems();
        setInterruptible(false);
    }

    @Override
    public void start(boolean hadToInterruptToStart) {
        directives[0].schedule();
    }

    @Override
    public void update() {
        // if the current directive is invalid or the whole procedure is done, do nothing
        if (currentDirectiveIndex < 0 || currentDirectiveIndex >= directives.length) {
            return;
        }

        // if current directive finished
        if (directives[currentDirectiveIndex].getHasFinished()) {
            // increase directive index
            currentDirectiveIndex++;

            if (currentDirectiveIndex >= directives.length) {
                return;
            }

            // schedule next directive
            directives[currentDirectiveIndex].schedule();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted && currentDirectiveIndex >= 0 && currentDirectiveIndex < directives.length) {
            directives[currentDirectiveIndex].stop(true);
        }
    }

    @Override
    public boolean isFinished() {
        return currentDirectiveIndex >= directives.length;
    }
}
