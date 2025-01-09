package org.firstinspires.ftc.teamcode;

public class RisingEdgeTrigger {
    private boolean previousInput = false;
    private boolean output = false;
    // Method to update the trigger state
    public void update(float currentInput) {
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
    public static void main(String[] args) {
        RisingEdgeTrigger trigger = new RisingEdgeTrigger();
        boolean input = false;
        // Simulate a while loop
        for (int i = 0; i < 10; i++) {
            // Simulate input change
            if (i == 3) {
                input = true;
            } else if (i == 4) {
                input = false;
            }
            // Update the trigger with the current input
            trigger.update(input);
            // Print the output state
            System.out.println("Cycle " + i + ": Output = " + trigger.wasTriggered());
        }
    }
}
