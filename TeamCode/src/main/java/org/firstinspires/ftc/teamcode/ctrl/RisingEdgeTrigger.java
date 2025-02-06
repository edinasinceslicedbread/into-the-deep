package org.firstinspires.ftc.teamcode.ctrl;

public class RisingEdgeTrigger {
    private boolean previousInput = false;
    private boolean output = false;
    // Method to update the trigger state
    public void update(boolean currentInput) {
        if (currentInput && !previousInput) {
            // Input has just gone true
            output = true;
        } else {
            // Reset output after one cycle
            output = false;
        }
        // Update previous input state
        previousInput = currentInput;
    }
    // Method to get the current output state
    public boolean wasTriggered() {
        return output;
    }
}
