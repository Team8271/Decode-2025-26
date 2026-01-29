package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.robot.configuration.Config;

import java.util.ArrayList;
import java.util.List;

public class AutoMaker {

    Config robot;
    Follower follower;

    public AutoMaker(Config robot, Follower follower){this.robot = robot; this.follower = follower;}

    /// AutoCommand interface
    public interface AutoCommand {}


    /// Pose Wrapper
    public static class PoseCmd implements AutoCommand {
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

    /// Helper for clean syntax
    public PathChainCmd PC(PathChain pathChain) {
        return new PathChainCmd(pathChain);
    }

    /// Action commands
    public enum ActionCmd implements AutoCommand {
        RUN_INTAKE,
        STOP_INTAKE,
        LAUNCH,
        IDLE;

    }

    public ActionCmd A(ActionCmd actionCmd) {return actionCmd;}

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

    public static class Sequence {
        private final List<AutoCommand> commands;

        public Sequence(List<AutoCommand> commands) {
            this.commands = commands;
        }

        public List<AutoCommand> getCommands() {
            return commands;
        }
    }


    /**
     * Builds a sequence of autonomous commands.
     * <p>
     * Usage pattern:
     * - First command MUST be a PoseCmd (for setting start position)
     * - Remaining commands can be pairs of PoseCmds (auto-generates PathChain between them)
     * - ActionCmds and PathChainCmds are added directly
     * <p>
     * Examples:
     * build(P(startPose), P(pose1), P(pose2)) → [startPose, path(pose1→pose2)]
     * build(P(startPose), P(pose1), A(RUN_INTAKE), P(pose2), P(pose3)) → [startPose, path(pose1→pose2), RUN_INTAKE, path(pose2→pose3)]
     * 
     * @param commands Auto Commands to run. Must start with PoseCmd.
     * @return A sequence ready to run using updateSequence()
     * @throws IllegalArgumentException if first command is not a PoseCmd
     */
    public Sequence build(AutoCommand... commands) {
        waitingForPath = false;
        waitingForLauncher = false;
        commandIndex = 0;

        List<AutoCommand> commandList = new ArrayList<AutoCommand>();

        if (commands.length == 0) {
            return new Sequence(commandList);
        }

        // First command must be PoseCmd for setting start position
        if (!(commands[0] instanceof PoseCmd)) {
            throw new IllegalArgumentException("First command must be a PoseCmd for setting start position");
        }

        // Add the first pose command directly (for setting start position)
        commandList.add(commands[0]);

        // Process remaining commands - look ahead for pose pairs
        // Start from i=1, but track the previous pose for pairing
        PoseCmd prevPose = null;
        for (int i = 1; i < commands.length; i++) {
            if (commands[i] instanceof PoseCmd) {
                PoseCmd currentPose = (PoseCmd) commands[i];
                
                if (prevPose != null) {
                    // We have a pair of poses - create PathChain
                    commandList.add(new PathChainCmd(follower.pathBuilder()
                            .addPath(new BezierCurve(prevPose.pose, currentPose.pose))
                            .setLinearHeadingInterpolation(prevPose.pose.getHeading(), currentPose.pose.getHeading())
                            .build()));
                    prevPose = null; // Reset for next pair
                } else {
                    // Store current pose as the first in a potential pair
                    prevPose = currentPose;
                }
            } else if (commands[i] instanceof ActionCmd || commands[i] instanceof PathChainCmd) {
                // If we have an unmatched pose, add it first
                if (prevPose != null) {
                    commandList.add(prevPose);
                    prevPose = null;
                }
                // Add ActionCmd or PathChainCmd directly
                commandList.add(commands[i]);
            }
        }
        
        // Add any remaining unmatched pose
        if (prevPose != null) {
            commandList.add(prevPose);
        }

        return new Sequence(commandList);
    }

    private boolean waitingForPath = false;
    private boolean waitingForLauncher = false;
    private int commandIndex = 0;

    /**
     * @implNote Call after each follower update in OpMode loop.
     * @param sequence a sequence of auto commands
     */
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
            commandIndex++;

        } else if (cmd instanceof PathChainCmd) {
            PathChainCmd pathCmd = (PathChainCmd) cmd;
            follower.followPath(pathCmd.pathChain);
            waitingForPath = true;
            commandIndex++;

        } else if (cmd instanceof ActionCmd) {
            ActionCmd actionCmd = (ActionCmd) cmd;
            runAction(actionCmd);
            if (actionCmd == ActionCmd.LAUNCH) {
                waitingForLauncher = true;
            }
            commandIndex++;
        }
    }
}
