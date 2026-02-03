/*package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.configuration.Config;

import java.io.File;
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

    public static class BlankSequence {
        private final List<AutoCommand> commands;

        public BlankSequence(List<AutoCommand> commands) {
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
     *
    public Sequence build(AutoCommand... commands) {
        waitingForPath = false;
        waitingForLauncher = false;
        commandIndex = 0;

        List<AutoCommand> commandList = new ArrayList<>();

        if (commands.length == 0) return new Sequence(commandList);

        if (!(commands[0] instanceof PoseCmd)) {
            throw new IllegalArgumentException("First command must be a PoseCmd");
        }

        // Start pose
        PoseCmd lastPose = (PoseCmd) commands[0];
        commandList.add(lastPose);

        // Process the rest
        for (int i = 1; i < commands.length; i++) {
            AutoCommand cmd = commands[i];

            if (cmd instanceof PoseCmd) {
                PoseCmd nextPose = (PoseCmd) cmd;

                // Build path immediately
                commandList.add(new PathChainCmd(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(lastPose.pose, nextPose.pose))
                                .setLinearHeadingInterpolation(
                                        lastPose.pose.getHeading(),
                                        nextPose.pose.getHeading()
                                )
                                .build()
                ));

                lastPose = nextPose; // carry forward
            }
            else {
                // ActionCmd or PathChainCmd
                commandList.add(cmd);
            }
        }

        Sequence seq = new Sequence(commandList);
        dumpSequence(seq);
        return seq;
    }

    public BlankSequence buildBlankSequence(AutoCommand... commands) {
        List<AutoCommand> commandList = new ArrayList<>();
        commandList.add((PoseCmd) commands[0]);

        for (int i = 1; i < commands.length; i++) {
            AutoCommand cmd = commands[i];

            if (cmd instanceof PoseCmd) {
                commandList.add((PoseCmd) commands[i]);
            }
            if (cmd instanceof ActionCmd) {
                commandList.add((ActionCmd) commands[i]);
            }

        }

        BlankSequence bSeq = new BlankSequence(commandList);
        return bSeq;
    }
/*
    public void printCase(BlankSequence bSeq) {
        StringBuilder code = new StringBuilder();

        List<AutoCommand> cmds = bSeq.getCommands();


        Pose startPose = poseCmdToPose((PoseCmd)cmds.get(0));
        String startPoseString = "new Pose(" + startPose.getX() + "," + startPose.getY() + "," + startPose.getHeading() + ")";

        code.append("package org.firstinspires.ftc.teamcode.robot.autons;\n" +
                "\n" +
                "import com.pedropathing.follower.Follower;\n" +
                "import com.pedropathing.geometry.BezierCurve;\n" +
                "import com.pedropathing.geometry.BezierLine;\n" +
                "import com.pedropathing.geometry.Pose;\n" +
                "import com.pedropathing.paths.Path;\n" +
                "import com.pedropathing.paths.PathChain;\n" +
                "import com.pedropathing.util.Timer;\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.OpMode;\n" +
                "import com.qualcomm.robotcore.util.ElapsedTime;\n" +
                "\n" +
                "import org.firstinspires.ftc.teamcode.pedroPathing.Constants;\n" +
                "import org.firstinspires.ftc.teamcode.robot.configuration.Config;\n" +
                "\n");
        code.append("@Autonomous(name = \"Blue Far Auto\")\n" +
                "public class ASDADSADADBlueFarAuto extends OpMode {\n" +
                "\n" +
                "    Config robot;\n" +
                "\n" +
                "    ElapsedTime runtime = new ElapsedTime();\n" +
                "\n" +
                "    boolean waitingForLauncher = false;\n" +
                "\n" +
                "    private Follower follower;\n" +
                "    \n" +
                "    private final Pose startPose = " + startPoseString + "; // Start Pose of robot.\n");

        robot.log(code.toString());
    }
*
    public void printCase(BlankSequence bSeq) {
        StringBuilder code = new StringBuilder();

        List<AutoCommand> cmds = bSeq.getCommands();

        Pose startPose = poseCmdToPose((PoseCmd)cmds.get(0));
        String startPoseString = poseToString(startPose);

        // Package and imports
        code.append("package org.firstinspires.ftc.teamcode.robot.autons;\n\n");
        code.append("import com.pedropathing.follower.Follower;\n");
        code.append("import com.pedropathing.geometry.BezierCurve;\n");
        code.append("import com.pedropathing.geometry.BezierLine;\n");
        code.append("import com.pedropathing.geometry.Pose;\n");
        code.append("import com.pedropathing.paths.Path;\n");
        code.append("import com.pedropathing.paths.PathChain;\n");
        code.append("import com.pedropathing.util.Timer;\n");
        code.append("import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n");
        code.append("import com.qualcomm.robotcore.eventloop.opmode.OpMode;\n");
        code.append("import com.qualcomm.robotcore.util.ElapsedTime;\n\n");
        code.append("import org.firstinspires.ftc.teamcode.pedroPathing.Constants;\n");
        code.append("import org.firstinspires.ftc.teamcode.robot.configuration.Config;\n\n");

        // Class declaration
        code.append("@Autonomous(name = \"Generated Auto\")\n");
        code.append("public class GeneratedAuto extends OpMode {\n\n");

        // Member variables
        code.append("    Config robot;\n");
        code.append("    ElapsedTime runtime = new ElapsedTime();\n");
        code.append("    boolean waitingForLauncher = false;\n");
        code.append("    private Follower follower;\n");
        code.append("    private Timer pathTimer, opmodeTimer;\n");
        code.append("    private int pathState;\n\n");

        // Start pose
        code.append("    private final Pose startPose = " + startPoseString + ";\n\n");

        // Generate pose variables
        int poseCounter = 1;
        for (int i = 1; i < cmds.size(); i++) {
            if (cmds.get(i) instanceof PoseCmd) {
                Pose pose = poseCmdToPose((PoseCmd) cmds.get(i));
                code.append("    private final Pose pose" + poseCounter + " = " + poseToString(pose) + ";\n");
                poseCounter++;
            }
        }
        code.append("\n");

        // Generate path variables
        code.append("    // Path variables\n");
        int pathCounter = 1;
        for (int i = 1; i < cmds.size(); i++) {
            if (cmds.get(i) instanceof PoseCmd) {
                code.append("    private PathChain path" + pathCounter + ";\n");
                pathCounter++;
            }
        }
        code.append("\n");

        // buildPaths() method
        code.append("    public void buildPaths() {\n");
        int pathNum = 1;
        int poseNum = 0;
        String lastPoseVar = "startPose";

        for (int i = 1; i < cmds.size(); i++) {
            if (cmds.get(i) instanceof PoseCmd) {
                poseNum++;
                String currentPoseVar = "pose" + poseNum;
                code.append("        path" + pathNum + " = follower.pathBuilder()\n");
                code.append("                .addPath(new BezierLine(" + lastPoseVar + ", " + currentPoseVar + "))\n");
                code.append("                .setLinearHeadingInterpolation(" + lastPoseVar + ".getHeading(), " + currentPoseVar + ".getHeading())\n");
                code.append("                .build();\n\n");
                lastPoseVar = currentPoseVar;
                pathNum++;
            }
        }
        code.append("    }\n\n");

        // autonomousPathUpdate() method
        code.append("    public void autonomousPathUpdate() throws InterruptedException {\n");
        code.append("        if (!robot.launcherThread.isBusy()) {\n");
        code.append("            waitingForLauncher = false;\n");
        code.append("        } else {\n");
        code.append("            return;\n");
        code.append("        }\n\n");
        code.append("        switch (pathState) {\n");

        int stateNum = 0;
        pathNum = 1;

        for (int i = 1; i < cmds.size(); i++) {
            AutoCommand cmd = cmds.get(i);

            if (cmd instanceof PoseCmd) {
                // State to follow path
                code.append("            case " + stateNum + ":\n");
                code.append("                if (!follower.isBusy()) {\n");
                code.append("                    follower.followPath(path" + pathNum + ", true);\n");
                code.append("                    setPathState(" + (stateNum + 1) + ");\n");
                code.append("                }\n");
                code.append("                break;\n");
                stateNum++;
                pathNum++;
            } else if (cmd instanceof ActionCmd) {
                ActionCmd action = (ActionCmd) cmd;

                // State to execute action
                code.append("            case " + stateNum + ":\n");
                code.append("                if (!follower.isBusy()) {\n");

                switch (action) {
                    case RUN_INTAKE:
                        code.append("                    robot.runIntakeAssembly(2000);\n");
                        break;
                    case STOP_INTAKE:
                        code.append("                    robot.stopIntakeAssembly();\n");
                        break;
                    case LAUNCH:
                        code.append("                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));\n");
                        code.append("                    robot.launcherThread.launchThree();\n");
                        break;
                    case IDLE:
                        code.append("                    robot.launcherThread.idleLauncher();\n");
                        break;
                }

                code.append("                    setPathState(" + (stateNum + 1) + ");\n");
                code.append("                }\n");
                code.append("                break;\n");
                stateNum++;
            }
        }

        code.append("        }\n");
        code.append("    }\n\n");

        // Helper methods
        code.append("    public void setPathState(int pState) {\n");
        code.append("        pathState = pState;\n");
        code.append("        pathTimer.resetTimer();\n");
        code.append("    }\n\n");

        // loop() method
        code.append("    @Override\n");
        code.append("    public void loop() {\n");
        code.append("        follower.update();\n");
        code.append("        try {\n");
        code.append("            autonomousPathUpdate();\n");
        code.append("        } catch (InterruptedException e) {\n");
        code.append("            throw new RuntimeException(e);\n");
        code.append("        }\n\n");
        code.append("        telemetry.addData(\"path state\", pathState);\n");
        code.append("        telemetry.addData(\"x\", follower.getPose().getX());\n");
        code.append("        telemetry.addData(\"y\", follower.getPose().getY());\n");
        code.append("        telemetry.addData(\"heading\", follower.getPose().getHeading());\n");
        code.append("        telemetry.update();\n");
        code.append("    }\n\n");

        // init() method
        code.append("    @Override\n");
        code.append("    public void init() {\n");
        code.append("        robot = new Config(this, follower);\n");
        code.append("        robot.init();\n");
        code.append("        robot.setOpModeIsActive(true);\n");
        code.append("        robot.setAlliance(Config.Alliance.BLUE);\n\n");
        code.append("        pathTimer = new Timer();\n");
        code.append("        opmodeTimer = new Timer();\n");
        code.append("        opmodeTimer.resetTimer();\n\n");
        code.append("        follower = Constants.createFollower(hardwareMap);\n");
        code.append("        buildPaths();\n");
        code.append("        follower.setStartingPose(startPose);\n");
        code.append("        robot.savePoseToFile(follower.getPose());\n");
        code.append("    }\n\n");

        // init_loop() method
        code.append("    @Override\n");
        code.append("    public void init_loop() {}\n\n");

        // start() method
        code.append("    @Override\n");
        code.append("    public void start() {\n");
        code.append("        runtime.reset();\n");
        code.append("        robot.setOpModeIsActive(true);\n");
        code.append("        opmodeTimer.resetTimer();\n");
        code.append("        setPathState(0);\n");
        code.append("    }\n\n");

        // stop() method
        code.append("    @Override\n");
        code.append("    public void stop() {\n");
        code.append("        robot.savePoseToFile(follower.getPose());\n");
        code.append("        robot.setOpModeIsActive(false);\n");
        code.append("    }\n");

        code.append("}\n");

        // Write to file with auto-incrementing filename
        writeGeneratedAutoToFile(code.toString());
    }

    private void writeGeneratedAutoToFile(String code) {
        try {
            // Create generatedAutons directory if it doesn't exist
            File generatedAutonsFolder = new File(AppUtil.FIRST_FOLDER, "generatedAutons");
            if (!generatedAutonsFolder.exists()) {
                generatedAutonsFolder.mkdirs();
            }

            // Find next available filename
            int fileNumber = 1;
            java.io.File autoFile;
            do {
                autoFile = new java.io.File(generatedAutonsFolder, "generatedAuto" + fileNumber + ".java");
                fileNumber++;
            } while (autoFile.exists());

            // Write the generated code
            java.io.BufferedWriter writer = new java.io.BufferedWriter(new java.io.FileWriter(autoFile));
            writer.write(code);
            writer.close();

            robot.log("[AMaker] Generated autonomous code written to " + autoFile.getAbsolutePath());

        } catch (java.io.IOException e) {
            robot.log("[AMaker] ERROR: Failed to write file: " + e.getMessage());
        }
    }

    private Pose poseCmdToPose(PoseCmd poseCmd) {
        return poseCmd.pose;
    }
    private String poseToString(Pose pose) {
        return "new Pose(" + pose.getX() + "," + pose.getY() + "," + pose.getHeading() + ")";
    }

    private void dumpSequence(Sequence sequence) {
        robot.log("[AMaker] ===== BUILT SEQUENCE =====");
        List<AutoCommand> cmds = sequence.getCommands();

        for (int i = 0; i < cmds.size(); i++) {
            AutoCommand cmd = cmds.get(i);

            if (cmd instanceof PoseCmd) {
                Pose p = ((PoseCmd) cmd).pose;
                robot.log(String.format(
                        "[AMaker] %02d: PoseCmd (%.1f, %.1f, %.1f)",
                        i, p.getX(), p.getY(), Math.toDegrees(p.getHeading())
                ));
            }
            else if (cmd instanceof PathChainCmd) {
                robot.log(String.format(
                        "[AMaker] %02d: PathChainCmd (%s)",
                        i, ((PathChainCmd) cmd).pathChain
                ));
            }
            else if (cmd instanceof ActionCmd) {
                robot.log(String.format(
                        "[AMaker] %02d: ActionCmd (%s)",
                        i, ((ActionCmd) cmd).name()
                ));
            }
            else {
                robot.log(String.format(
                        "[AMaker] %02d: UNKNOWN CMD (%s)",
                        i, cmd.getClass().getSimpleName()
                ));
            }
        }

        robot.log("[AMaker] ==========================");
    }


    private boolean waitingForPath = false;
    private boolean waitingForLauncher = false;
    private int commandIndex = 0;

    public int getCommandIndex() {return commandIndex;}

    /**
     * @implNote Call after each follower update in OpMode loop.
     * @param sequence a sequence of auto commands
     *
    public void updateSequence(Sequence sequence) {
        log("update");
        log("cmd index = " + commandIndex);

        // Don’t advance if waiting on something
        if (waitingForPath && follower.isBusy()) {
            log("Waiting for path");
            return;
        }
        if (waitingForLauncher && robot.launcherThread.isBusy()) {
            log("Waiting for launcher");
            return;
        }

        // Clear waits once done
        waitingForPath = false;
        waitingForLauncher = false;

        if (commandIndex >= sequence.getCommands().size()) {
            log("Finished all commands");
            return;
        }

        AutoCommand cmd = sequence.getCommands().get(commandIndex);

        if (cmd instanceof PoseCmd) { // StartPose or Pose Resetting
            PoseCmd poseCmd = (PoseCmd) cmd;
            follower.setPose(poseCmd.pose);
            log("Starting pose set to " + poseCmd.pose);
            commandIndex++;

        } else if (cmd instanceof PathChainCmd) {
            PathChainCmd pathCmd = (PathChainCmd) cmd;
            follower.followPath(pathCmd.pathChain);
            log("Following path " + pathCmd.pathChain.toString());
            waitingForPath = true;
            commandIndex++;

        } else if (cmd instanceof ActionCmd) {
            ActionCmd actionCmd = (ActionCmd) cmd;
            runAction(actionCmd);
            log("Running action " + actionCmd.name());
            if (actionCmd == ActionCmd.LAUNCH) {
                waitingForLauncher = true;
            }
            commandIndex++;
        }
    }

    private void log(String msg) {
        robot.log("[AMaker] " + msg);
    }
}
*/

package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.configuration.Config;

import java.io.File;
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
        public final String name;

        public PoseCmd(Pose pose, String name) {
            this.pose = pose;
            this.name = name;
        }

        public PoseCmd(Pose pose) {
            this.pose = pose;
            this.name = null;
        }
    }


    /// Helper for clean syntax
    public PoseCmd P(Pose pose) {
        return new PoseCmd(pose, getPoseName(pose));
    }

    public PoseCmd P(Pose pose, String name) {
        return new PoseCmd(pose, name);
    }

    /**
     * Automatically detects the pose name from the Poses class using reflection
     */
    private String getPoseName(Pose pose) {
        try {
            // Check Blue poses
            java.lang.reflect.Field[] blueFields = org.firstinspires.ftc.teamcode.util.Poses.Blue.class.getDeclaredFields();
            for (java.lang.reflect.Field field : blueFields) {
                if (field.getType() == Pose.class) {
                    field.setAccessible(true);
                    Pose fieldPose = (Pose) field.get(null);
                    if (fieldPose == pose) {
                        return "Blue." + field.getName();
                    }
                }
            }

            // Check Red poses
            java.lang.reflect.Field[] redFields = org.firstinspires.ftc.teamcode.util.Poses.Red.class.getDeclaredFields();
            for (java.lang.reflect.Field field : redFields) {
                if (field.getType() == Pose.class) {
                    field.setAccessible(true);
                    Pose fieldPose = (Pose) field.get(null);
                    if (fieldPose == pose) {
                        return "Red." + field.getName();
                    }
                }
            }
        } catch (Exception e) {
            // Reflection failed, return null
        }

        return null; // Name not found
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

    public static class BlankSequence {
        private final List<AutoCommand> commands;

        public BlankSequence(List<AutoCommand> commands) {
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

        List<AutoCommand> commandList = new ArrayList<>();

        if (commands.length == 0) return new Sequence(commandList);

        if (!(commands[0] instanceof PoseCmd)) {
            throw new IllegalArgumentException("First command must be a PoseCmd");
        }

        // Start pose
        PoseCmd lastPose = (PoseCmd) commands[0];
        commandList.add(lastPose);

        // Process the rest
        for (int i = 1; i < commands.length; i++) {
            AutoCommand cmd = commands[i];

            if (cmd instanceof PoseCmd) {
                PoseCmd nextPose = (PoseCmd) cmd;

                // Build path immediately
                commandList.add(new PathChainCmd(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(lastPose.pose, nextPose.pose))
                                .setLinearHeadingInterpolation(
                                        lastPose.pose.getHeading(),
                                        nextPose.pose.getHeading()
                                )
                                .build()
                ));

                lastPose = nextPose; // carry forward
            }
            else {
                // ActionCmd or PathChainCmd
                commandList.add(cmd);
            }
        }

        Sequence seq = new Sequence(commandList);
        dumpSequence(seq);
        return seq;
    }

    public BlankSequence buildBlankSequence(AutoCommand... commands) {
        List<AutoCommand> commandList = new ArrayList<>();
        commandList.add((PoseCmd) commands[0]);

        for (int i = 1; i < commands.length; i++) {
            AutoCommand cmd = commands[i];

            if (cmd instanceof PoseCmd) {
                commandList.add((PoseCmd) commands[i]);
            }
            if (cmd instanceof ActionCmd) {
                commandList.add((ActionCmd) commands[i]);
            }

        }

        BlankSequence bSeq = new BlankSequence(commandList);
        return bSeq;
    }

    public void printCase(BlankSequence bSeq) {
        StringBuilder code = new StringBuilder();

        List<AutoCommand> cmds = bSeq.getCommands();

        PoseCmd startPoseCmd = (PoseCmd)cmds.get(0);
        Pose startPose = poseCmdToPose(startPoseCmd);
        String startPoseString = poseToString(startPose);

        // Package and imports
        code.append("package org.firstinspires.ftc.teamcode.robot.autons;\n\n");
        code.append("import com.pedropathing.follower.Follower;\n");
        code.append("import com.pedropathing.geometry.BezierCurve;\n");
        code.append("import com.pedropathing.geometry.BezierLine;\n");
        code.append("import com.pedropathing.geometry.Pose;\n");
        code.append("import com.pedropathing.paths.Path;\n");
        code.append("import com.pedropathing.paths.PathChain;\n");
        code.append("import com.pedropathing.util.Timer;\n");
        code.append("import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n");
        code.append("import com.qualcomm.robotcore.eventloop.opmode.OpMode;\n");
        code.append("import com.qualcomm.robotcore.util.ElapsedTime;\n\n");
        code.append("import org.firstinspires.ftc.teamcode.pedroPathing.Constants;\n");
        code.append("import org.firstinspires.ftc.teamcode.robot.configuration.Config;\n");
        code.append("import org.firstinspires.ftc.teamcode.util.Poses;\n\n");

        // Class declaration
        code.append("@Autonomous(name = \"Generated Auto\")\n");
        code.append("public class GeneratedAuto extends OpMode {\n\n");

        // Member variables
        code.append("    Config robot;\n");
        code.append("    ElapsedTime runtime = new ElapsedTime();\n");
        code.append("    boolean waitingForLauncher = false;\n");
        code.append("    private Follower follower;\n");
        code.append("    private Timer pathTimer, opmodeTimer;\n");
        code.append("    private int pathState;\n\n");

        // Start pose - use Poses class reference if available
        String startPoseName = startPoseCmd.name != null ? startPoseCmd.name : "startPose";
        boolean useDirectPoseReference = startPoseCmd.name != null && startPoseCmd.name.contains(".");

        if (useDirectPoseReference) {
            code.append("    private final Pose " + cleanPoseName(startPoseName) + " = Poses." + startPoseName + ";\n\n");
        } else {
            code.append("    private final Pose " + startPoseName + " = " + startPoseString + ";\n\n");
        }

        // Generate pose variables with names
        int poseCounter = 1;
        List<String> poseNames = new ArrayList<>();
        poseNames.add(useDirectPoseReference ? cleanPoseName(startPoseName) : startPoseName); // Add start pose name

        for (int i = 1; i < cmds.size(); i++) {
            if (cmds.get(i) instanceof PoseCmd) {
                PoseCmd poseCmd = (PoseCmd) cmds.get(i);
                Pose pose = poseCmdToPose(poseCmd);
                String poseName;
                boolean usePosesRef = poseCmd.name != null && poseCmd.name.contains(".");

                if (usePosesRef) {
                    poseName = cleanPoseName(poseCmd.name);
                    code.append("    private final Pose " + poseName + " = Poses." + poseCmd.name + ";\n");
                } else {
                    poseName = poseCmd.name != null ? poseCmd.name : ("pose" + poseCounter);
                    code.append("    private final Pose " + poseName + " = " + poseToString(pose) + ";\n");
                }

                poseNames.add(poseName);
                poseCounter++;
            }
        }
        code.append("\n");

        // Generate path variables with descriptive names
        code.append("    // Path variables\n");
        int pathCounter = 1;
        int poseIndex = 0;
        List<String> pathNames = new ArrayList<>();

        for (int i = 1; i < cmds.size(); i++) {
            if (cmds.get(i) instanceof PoseCmd) {
                String fromPose = poseNames.get(poseIndex);
                String toPose = poseNames.get(poseIndex + 1);
                String pathName = fromPose + "_to_" + toPose;
                pathNames.add(pathName);
                code.append("    private PathChain " + pathName + ";\n");
                pathCounter++;
                poseIndex++;
            }
        }
        code.append("\n");

        // buildPaths() method
        code.append("    public void buildPaths() {\n");
        int pathNum = 0;
        poseIndex = 0;

        for (int i = 1; i < cmds.size(); i++) {
            if (cmds.get(i) instanceof PoseCmd) {
                String lastPoseVar = poseNames.get(poseIndex);
                String currentPoseVar = poseNames.get(poseIndex + 1);
                String pathName = pathNames.get(pathNum);

                code.append("        " + pathName + " = follower.pathBuilder()\n");
                code.append("                .addPath(new BezierLine(" + lastPoseVar + ", " + currentPoseVar + "))\n");
                code.append("                .setLinearHeadingInterpolation(" + lastPoseVar + ".getHeading(), " + currentPoseVar + ".getHeading())\n");
                code.append("                .build();\n\n");
                poseIndex++;
                pathNum++;
            }
        }
        code.append("    }\n\n");

        // autonomousPathUpdate() method
        code.append("    public void autonomousPathUpdate() throws InterruptedException {\n");
        code.append("        if (!robot.launcherThread.isBusy()) {\n");
        code.append("            waitingForLauncher = false;\n");
        code.append("        } else {\n");
        code.append("            return;\n");
        code.append("        }\n\n");
        code.append("        switch (pathState) {\n");

        int stateNum = 0;
        pathNum = 0;

        for (int i = 1; i < cmds.size(); i++) {
            AutoCommand cmd = cmds.get(i);

            if (cmd instanceof PoseCmd) {
                // State to follow path
                String pathName = pathNames.get(pathNum);
                code.append("            case " + stateNum + ": // Follow " + pathName + "\n");
                code.append("                if (!follower.isBusy()) {\n");
                code.append("                    follower.followPath(" + pathName + ", true);\n");
                code.append("                    setPathState(" + (stateNum + 1) + ");\n");
                code.append("                }\n");
                code.append("                break;\n");
                stateNum++;
                pathNum++;
            } else if (cmd instanceof ActionCmd) {
                ActionCmd action = (ActionCmd) cmd;

                // State to execute action
                code.append("            case " + stateNum + ": // " + action.name() + "\n");
                code.append("                if (!follower.isBusy()) {\n");

                switch (action) {
                    case RUN_INTAKE:
                        code.append("                    robot.runIntakeAssembly(2000);\n");
                        break;
                    case STOP_INTAKE:
                        code.append("                    robot.stopIntakeAssembly();\n");
                        break;
                    case LAUNCH:
                        code.append("                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));\n");
                        code.append("                    robot.launcherThread.launchThree();\n");
                        break;
                    case IDLE:
                        code.append("                    robot.launcherThread.idleLauncher();\n");
                        break;
                }

                code.append("                    setPathState(" + (stateNum + 1) + ");\n");
                code.append("                }\n");
                code.append("                break;\n");
                stateNum++;
            }
        }

        code.append("        }\n");
        code.append("    }\n\n");

        // Helper methods
        code.append("    public void setPathState(int pState) {\n");
        code.append("        pathState = pState;\n");
        code.append("        pathTimer.resetTimer();\n");
        code.append("    }\n\n");

        // loop() method
        code.append("    @Override\n");
        code.append("    public void loop() {\n");
        code.append("        follower.update();\n");
        code.append("        try {\n");
        code.append("            autonomousPathUpdate();\n");
        code.append("        } catch (InterruptedException e) {\n");
        code.append("            throw new RuntimeException(e);\n");
        code.append("        }\n\n");
        code.append("        telemetry.addData(\"path state\", pathState);\n");
        code.append("        telemetry.addData(\"x\", follower.getPose().getX());\n");
        code.append("        telemetry.addData(\"y\", follower.getPose().getY());\n");
        code.append("        telemetry.addData(\"heading\", follower.getPose().getHeading());\n");
        code.append("        telemetry.update();\n");
        code.append("    }\n\n");

        // init() method
        code.append("    @Override\n");
        code.append("    public void init() {\n");
        code.append("        robot = new Config(this, follower);\n");
        code.append("        robot.init();\n");
        code.append("        robot.setOpModeIsActive(true);\n");
        code.append("        robot.setAlliance(Config.Alliance.BLUE);\n\n");
        code.append("        pathTimer = new Timer();\n");
        code.append("        opmodeTimer = new Timer();\n");
        code.append("        opmodeTimer.resetTimer();\n\n");
        code.append("        follower = Constants.createFollower(hardwareMap);\n");
        code.append("        buildPaths();\n");
        String startPoseVarName = useDirectPoseReference ? cleanPoseName(startPoseName) : startPoseName;
        code.append("        follower.setStartingPose(" + startPoseVarName + ");\n");
        code.append("        robot.savePoseToFile(follower.getPose());\n");
        code.append("    }\n\n");

        // init_loop() method
        code.append("    @Override\n");
        code.append("    public void init_loop() {}\n\n");

        // start() method
        code.append("    @Override\n");
        code.append("    public void start() {\n");
        code.append("        runtime.reset();\n");
        code.append("        robot.setOpModeIsActive(true);\n");
        code.append("        opmodeTimer.resetTimer();\n");
        code.append("        setPathState(0);\n");
        code.append("    }\n\n");

        // stop() method
        code.append("    @Override\n");
        code.append("    public void stop() {\n");
        code.append("        robot.savePoseToFile(follower.getPose());\n");
        code.append("        robot.setOpModeIsActive(false);\n");
        code.append("    }\n");

        code.append("}\n");

        // Write to file with auto-incrementing filename
        writeGeneratedAutoToFile(code.toString());
    }

    private void writeGeneratedAutoToFile(String code) {
        try {
            // Create generatedAutons directory if it doesn't exist
            File generatedAutonsFolder = new File(AppUtil.FIRST_FOLDER, "generatedAutons");
            if (!generatedAutonsFolder.exists()) {
                generatedAutonsFolder.mkdirs();
            }

            // Find next available filename
            int fileNumber = 1;
            java.io.File autoFile;
            do {
                autoFile = new java.io.File(generatedAutonsFolder, "generatedAuto" + fileNumber + ".java");
                fileNumber++;
            } while (autoFile.exists());

            // Write the generated code
            java.io.BufferedWriter writer = new java.io.BufferedWriter(new java.io.FileWriter(autoFile));
            writer.write(code);
            writer.close();

            robot.log("[AMaker] Generated autonomous code written to " + autoFile.getAbsolutePath());

        } catch (java.io.IOException e) {
            robot.log("[AMaker] ERROR: Failed to write file: " + e.getMessage());
        }
    }

    private Pose poseCmdToPose(PoseCmd poseCmd) {
        return poseCmd.pose;
    }

    private String poseToString(Pose pose) {
        return "new Pose(" + pose.getX() + ", " + pose.getY() + ", " + pose.getHeading() + ")";
    }

    /**
     * Cleans pose names like "Blue.farStart" to just "farStart" for variable names
     */
    private String cleanPoseName(String poseName) {
        if (poseName.contains(".")) {
            return poseName.substring(poseName.lastIndexOf('.') + 1);
        }
        return poseName;
    }

    private void dumpSequence(Sequence sequence) {
        robot.log("[AMaker] ===== BUILT SEQUENCE =====");
        List<AutoCommand> cmds = sequence.getCommands();

        for (int i = 0; i < cmds.size(); i++) {
            AutoCommand cmd = cmds.get(i);

            if (cmd instanceof PoseCmd) {
                Pose p = ((PoseCmd) cmd).pose;
                robot.log(String.format(
                        "[AMaker] %02d: PoseCmd (%.1f, %.1f, %.1f)",
                        i, p.getX(), p.getY(), Math.toDegrees(p.getHeading())
                ));
            }
            else if (cmd instanceof PathChainCmd) {
                robot.log(String.format(
                        "[AMaker] %02d: PathChainCmd (%s)",
                        i, ((PathChainCmd) cmd).pathChain
                ));
            }
            else if (cmd instanceof ActionCmd) {
                robot.log(String.format(
                        "[AMaker] %02d: ActionCmd (%s)",
                        i, ((ActionCmd) cmd).name()
                ));
            }
            else {
                robot.log(String.format(
                        "[AMaker] %02d: UNKNOWN CMD (%s)",
                        i, cmd.getClass().getSimpleName()
                ));
            }
        }

        robot.log("[AMaker] ==========================");
    }


    private boolean waitingForPath = false;
    private boolean waitingForLauncher = false;
    private int commandIndex = 0;

    public int getCommandIndex() {return commandIndex;}

    /**
     * @implNote Call after each follower update in OpMode loop.
     * @param sequence a sequence of auto commands
     */
    public void updateSequence(Sequence sequence) {
        log("update");
        log("cmd index = " + commandIndex);

        // Don't advance if waiting on something
        if (waitingForPath && follower.isBusy()) {
            log("Waiting for path");
            return;
        }
        if (waitingForLauncher && robot.launcherThread.isBusy()) {
            log("Waiting for launcher");
            return;
        }

        // Clear waits once done
        waitingForPath = false;
        waitingForLauncher = false;

        if (commandIndex >= sequence.getCommands().size()) {
            log("Finished all commands");
            return;
        }

        AutoCommand cmd = sequence.getCommands().get(commandIndex);

        if (cmd instanceof PoseCmd) { // StartPose or Pose Resetting
            PoseCmd poseCmd = (PoseCmd) cmd;
            follower.setPose(poseCmd.pose);
            log("Starting pose set to " + poseCmd.pose);
            commandIndex++;

        } else if (cmd instanceof PathChainCmd) {
            PathChainCmd pathCmd = (PathChainCmd) cmd;
            follower.followPath(pathCmd.pathChain);
            log("Following path " + pathCmd.pathChain.toString());
            waitingForPath = true;
            commandIndex++;

        } else if (cmd instanceof ActionCmd) {
            ActionCmd actionCmd = (ActionCmd) cmd;
            runAction(actionCmd);
            log("Running action " + actionCmd.name());
            if (actionCmd == ActionCmd.LAUNCH) {
                waitingForLauncher = true;
            }
            commandIndex++;
        }
    }

    private void log(String msg) {
        robot.log("[AMaker] " + msg);
    }
}