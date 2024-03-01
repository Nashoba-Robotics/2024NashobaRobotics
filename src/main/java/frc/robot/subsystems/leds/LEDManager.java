package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Governor;
import frc.robot.RobotContainer;

public class LEDManager extends SubsystemBase{
    private static CANdle candle;

    public LEDManager(){
        candle = new CANdle(0, "jerry");

        candle.configFactoryDefault();
    }

    @Override
    public void periodic() {
        switch (Governor.getRobotState()) {
            case NEUTRAL:
                if(RobotContainer.loader.getShooterSensor()) setColor(new Color(255, 255, 255));
                else new Color(255, 0, 0);
                break;
        
            default:
                break;
        }
    }

    public void setColor(Color color){
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

    public static enum LEDState{
        DISABLED,
        NEUTRAL,
        INTAKE,
        SOURCE,
        SHOOT_PREP,
        SHOOT,
        HAVE_NOTE
    }
}
