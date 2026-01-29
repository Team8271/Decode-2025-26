package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.robot.Config;
import org.firstinspires.ftc.teamcode.robot.LauncherThread;

import java.util.List;

public class AutoMaker {

    Config robot;
    Follower follower;

    public AutoMaker(Config robot, Follower follower){this.robot = robot; this.follower = follower;}

    /// AutoCommand interface
    public interface AutoCommand {}


    /// Pose Wrapper
    public class PoseCmd implements AutoCommand {
        public final Pose pose;

        public PoseCmd(Pose pose) {
            this.pose = pose;
        }
    }


    /// Helper for clean syntax
    public PoseCmd P(Pose pose) {
        return new PoseCmd(pose);
    }

    public static class PathChainCmd implements AutoCommand {
        public final PathChain pathChain;

        public PathChainCmd(PathChain pathChain) {
            this.pathChain = pathChain;
        }
    }

    /// Action commands
    public enum ActionCmd implements AutoCommand {
        RUN_INTAKE,
        STOP_INTAKE,
        LAUNCH,
        IDLE;

        /// Helper for clean syntax
        public static ActionCmd A(ActionCmd actionCmd) {
            return actionCmd;
        }
    }

    /// Action executor
    private void runAction(ActionCmd action) {
        switch (action) {
            case RUN_INTAKE:
                robot.runIntakeAssembly(2000);
                break;
            case STOP_INTAKE:
                robot.stopIntakeAssembly();
                break;
            case LAUNCH:
                robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                robot.launcherThread.launchThree();
                break;
            case IDLE:
                robot.launcherThread.idleLauncher();
                break;
        }
    }

    public interface BuiltCommands {}

    public class Sequence {
        private final List<AutoCommand> commands;

        public Sequence(List<AutoCommand> commands) {
            this.commands = commands;
        }

        public List<AutoCommand> getCommands() {
            return commands;
        }
    }



    public Sequence build(AutoCommand... commands) {
        return new Sequence(List.of(commands));
    }

    boolean waitingForPath = false;
    boolean waitingForLauncher = false;
    int commandIndex = 0;
    public void updateSequence(Sequence sequence) {

        // Don’t advance if waiting on something
        if (waitingForPath && follower.isBusy()) return;
        if (waitingForLauncher && robot.launcherThread.isBusy()) return;

        // Clear waits once done
        waitingForPath = false;
        waitingForLauncher = false;

        if (commandIndex >= sequence.getCommands().size()) return;

        AutoCommand cmd = sequence.getCommands().get(commandIndex);

        if (cmd instanceof PoseCmd) { // StartPose or Pose Resetting
            PoseCmd poseCmd = (PoseCmd) cmd;
            follower.setPose(poseCmd.pose);

        } else if (cmd instanceof PathChainCmd) {
            PathChainCmd pathCmd = (PathChainCmd) cmd;
            follower.followPath(pathCmd.pathChain);
            // Set waitforpathFinish to true

        } else if (cmd instanceof ActionCmd) {
            ActionCmd actionCmd = (ActionCmd) cmd;
            runAction(actionCmd);
            if (cmd == ActionCmd.LAUNCH) {
                // Set waitForLauncherFinish true if launching
            }
        }
    }
}
