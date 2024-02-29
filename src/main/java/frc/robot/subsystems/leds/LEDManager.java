package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;

public class LEDManager {
    private static CANdle candle;

    public LEDManager(){
        candle = new CANdle(0, "jerry");

        candle.configFactoryDefault();
    }

    public static void setColor(Color color){
        candle.setLEDs(color.r, color.g, color.b);
    }

    public static class Color{
        int r, g, b;
        public Color(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
}
