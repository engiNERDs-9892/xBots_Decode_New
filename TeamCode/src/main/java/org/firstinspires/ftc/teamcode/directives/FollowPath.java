package org.firstinspires.ftc.teamcode.directives;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.stellarstructure.runnables.Directive;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

public class FollowPath extends Directive {
    private PathChain path;
    private Follower follower;

    private boolean holdEnd;

    public FollowPath(PathChain path, Follower follower, boolean holdEnd) {
        setInterruptible(true);
        setRequires(Drivebase.getInstance());
        this.path = path;
        this.follower = follower;
        this.holdEnd = holdEnd;
    }

    @Override
    public void start(boolean hadToInterruptToStart) {
        follower.followPath(path, holdEnd);
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public void stop(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
