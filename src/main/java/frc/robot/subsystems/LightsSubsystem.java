package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {

    // Note: on Shifty, the lights are Y-ed, both of the strips run the same pattern.

    private final AddressableLED       ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private int                        rainbowFirstPixelHue = 0;

    private static final int           LED_STRING_LENGTH    = 54;
    private static final int           RAINBOW_LENGTH       = 24;
    private static final int           PROXIMITY_PIXEL      = LED_STRING_LENGTH - 2;

    private static final Color         NEUTRAL              = new Color(60, 60, 60);
    private static final Color         FORWARD              = new Color(0, 90, 0);
    private static final Color         REVERSE              = new Color(80, 0, 0);

    /** Creates a new DriveSubsystem. */
    public LightsSubsystem() {

        ledStrip  = new AddressableLED(9);
        ledBuffer = new AddressableLEDBuffer(LED_STRING_LENGTH);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);

        ledStrip.start();
    }

    public void setProximity(boolean value) {
        // The proximity light is the last light in the LED chain.
        // Set it to red (dim) if false, and bright green if true.
        if (value) {
            ledBuffer.setRGB(PROXIMITY_PIXEL, 0, 90, 0);
        }
        else {
            ledBuffer.setRGB(PROXIMITY_PIXEL, 15, 0, 0);
        }
    }

    public void setMotorSpeeds(double left, double right) {

        for (int i = 0; i < RAINBOW_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
            ledBuffer.setRGB(RAINBOW_LENGTH * 2 - i, 0, 0, 0);
        }

        int   leftValue  = (int) ((1.0 - left) / 2.0d * RAINBOW_LENGTH);
        int   rightValue = (int) ((1.0 - right) / 2.0d * RAINBOW_LENGTH);

        Color leftColor  = NEUTRAL;
        if (left > 0) {
            leftColor = FORWARD;
        }
        else if (left < 0) {
            leftColor = REVERSE;
        }
        ledBuffer.setLED(RAINBOW_LENGTH * 2 - 1 - leftValue, leftColor);

        Color rightColor = NEUTRAL;
        if (right > 0) {
            rightColor = FORWARD;
        }
        else if (right < 0) {
            rightColor = REVERSE;
        }
        ledBuffer.setLED(rightValue, rightColor);

    }

    private void updateRainbow() {

        // For every pixel
        for (var i = 0; i < RAINBOW_LENGTH; i++) {

            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;

            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
            // Set the corresponding pixel on the opposite side.
            ledBuffer.setHSV(RAINBOW_LENGTH * 2 - 1 - i, hue, 255, 128);
        }

        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    @Override
    public void periodic() {

        // updateRainbow();
        ledStrip.setData(ledBuffer);

        SmartDashboard.putNumber("Hue", rainbowFirstPixelHue);
    }

}
