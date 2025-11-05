package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Multi-stage selector for FTC autonomous configuration.
 * Allows sequential selection of options during init phase.
 * <p>
 * Example usage:
 * Selector selector = new Selector.Builder(this)
 *     .addStage(new Selector.Stage("Alliance")
 *         .addOption("RED", "b")
 *         .addOption("BLUE", "x"))
 *     .addStage(new Selector.Stage("Starting Position")
 *         .addOption("LEFT", "dpad_left")
 *         .addOption("CENTER", "a")
 *         .addOption("RIGHT", "dpad_right"))
 *     .build();
 * <p>
 * Map<String, String> results = selector.select();
 * String alliance = selector.getSelection("Alliance");
 */
public class Selector {

    /**
     * Represents a single selectable option with a name and button mapping.
     */
    public static class Option {
        private String name;
        private String displayName;
        private String inputButton;

        public Option(String name, String button) {
            this.name = name;
            this.displayName = name;
            this.inputButton = button;
        }

        public Option setDisplayName(String displayName) {
            this.displayName = displayName;
            return this;
        }

        public String getName() {
            return name;
        }

        public String getDisplayName() {
            return displayName;
        }

        public String getButton() {
            return inputButton;
        }
    }

    /**
     * Represents a selection stage with multiple options.
     */
    public static class Stage {
        private String title;
        private List<Option> options = new ArrayList<>();

        public Stage(String title) {
            this.title = title;
        }

        public Stage addOption(String name, String button) {
            options.add(new Option(name, button));
            return this;
        }

        public Stage addOption(Option option) {
            options.add(option);
            return this;
        }

        public String getTitle() {
            return title;
        }

        public List<Option> getOptions() {
            return options;
        }
    }

    /**
     * Builder for creating a Selector instance.
     */
    public static class Builder {
        private LinearOpMode opMode = null;
        private List<Stage> stages = new ArrayList<>();
        private Gamepad gamepad = null;
        private long confirmationDelayMs = 500;

        public Builder(LinearOpMode opMode) {
            this.opMode = opMode;
            this.gamepad = opMode.gamepad1;
        }

        public Builder setGamepad(Gamepad gamepad) {
            this.gamepad = gamepad;
            return this;
        }

        public Builder setConfirmationDelay(long milliseconds) {
            this.confirmationDelayMs = milliseconds;
            return this;
        }

        public Builder addStage(Stage stage) {
            stages.add(stage);
            return this;
        }

        public Selector build() {
            if (opMode == null) {
                throw new IllegalStateException("LinearOpMode must be set");
            }
            if (stages.isEmpty()) {
                throw new IllegalStateException("At least one stage must be added");
            }
            return new Selector(opMode, stages, gamepad, confirmationDelayMs);
        }
    }

    private LinearOpMode opMode;
    private List<Stage> stages;
    private Gamepad gamepad;
    private long confirmationDelayMs;
    private Map<String, String> selections = new HashMap<>();

    private Selector(LinearOpMode opMode, List<Stage> stages, Gamepad gamepad, long confirmationDelayMs) {
        this.opMode = opMode;
        this.stages = stages;
        this.gamepad = gamepad;
        this.confirmationDelayMs = confirmationDelayMs;
    }

    /**
     * Runs the selection process, prompting for each stage sequentially.
     * @return Map of stage titles to selected option names
     */
    public Map<String, String> select() {
        selections.clear();

        // Go through each stage one by one
        for (Stage stage : stages) {
            String selected = selectFromStage(stage);
            if (selected == null) {
                // If opMode is no longer in init, use first option as default
                if (!stage.getOptions().isEmpty()) {
                    selected = stage.getOptions().get(0).getName();
                }
            }
            selections.put(stage.getTitle(), selected);
        }

        // Show final summary
        showSummary();

        return selections;
    }

    /**
     * Handles selection for a single stage.
     */
    private String selectFromStage(Stage stage) {
        boolean[] buttonPressed = new boolean[stage.getOptions().size()];
        String selectedOption = null;

        while (opMode.opModeInInit() && selectedOption == null) {
            // Display current selections so far
            opMode.telemetry.clear();

            if (!selections.isEmpty()) {
                opMode.telemetry.addLine("=== Previous Selections ===");
                for (Map.Entry<String, String> prev : selections.entrySet()) {
                    opMode.telemetry.addData(prev.getKey(), prev.getValue());
                }
                opMode.telemetry.addLine("");
            }

            // Display current stage
            opMode.telemetry.addLine("=== " + stage.getTitle() + " ===");
            opMode.telemetry.addLine("");

            for (Option option : stage.getOptions()) {
                opMode.telemetry.addData("[" + option.getButton().toUpperCase() + "]", option.getDisplayName());
            }

            opMode.telemetry.update();

            // Check for button presses
            for (int i = 0; i < stage.getOptions().size(); i++) {
                Option option = stage.getOptions().get(i);
                boolean currentlyPressed = isButtonPressed(option.getButton());

                if (currentlyPressed && !buttonPressed[i]) {
                    selectedOption = option.getName();

                    // Show confirmation
                    opMode.telemetry.clear();
                    opMode.telemetry.addLine("✓ Selected: " + option.getDisplayName());
                    opMode.telemetry.update();
                    opMode.sleep(confirmationDelayMs);

                    return selectedOption;
                }

                buttonPressed[i] = currentlyPressed;
            }

            opMode.sleep(50);
        }

        return selectedOption;
    }

    /**
     * Displays final configuration summary.
     */
    private void showSummary() {
        opMode.telemetry.clear();
        opMode.telemetry.addLine("=== FINAL CONFIGURATION ===");
        opMode.telemetry.addLine("");

        for (Map.Entry<String, String> selection : selections.entrySet()) {
            opMode.telemetry.addData(selection.getKey(), selection.getValue());
        }

        opMode.telemetry.addLine("");
        opMode.telemetry.addLine("Ready to Start!");
        opMode.telemetry.update();
    }

    /**
     * Gets the selected option for a specific stage.
     * @param stageTitle The title of the stage
     * @return The name of the selected option
     */
    public String getSelection(String stageTitle) {
        return selections.get(stageTitle);
    }

    /**
     * Gets all selections as a map.
     * @return Map of stage titles to selected option names
     */
    public Map<String, String> getAllSelections() {
        return new HashMap<>(selections);
    }

    /**
     * Checks if a gamepad button is currently pressed.
     */
    private boolean isButtonPressed(String button) {
        switch (button.toLowerCase()) {
            case "a": return gamepad.a;
            case "b": return gamepad.b;
            case "x": return gamepad.x;
            case "y": return gamepad.y;
            case "dpad_up": return gamepad.dpad_up;
            case "dpad_down": return gamepad.dpad_down;
            case "dpad_left": return gamepad.dpad_left;
            case "dpad_right": return gamepad.dpad_right;
            case "left_bumper": return gamepad.left_bumper;
            case "right_bumper": return gamepad.right_bumper;
            case "start": return gamepad.start;
            case "back": return gamepad.back;
            default: return false;
        }
    }
}