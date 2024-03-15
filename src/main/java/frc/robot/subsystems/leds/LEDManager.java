package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.sensors.SensorManager;

public class LEDManager extends SubsystemBase{
    private static CANdle candle;
    private RobotState lastState;

    private static final int LED_COUNT = 85;

    private boolean clearAnimationFlag = false;

    public LEDManager(){
        candle = new CANdle(0, "jerry");

        candle.configFactoryDefault();
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGBW;
        config.brightnessScalar = 1;


        lastState = RobotState.UNKNOWN;
    }

    @Override
    public void periodic() {
        RobotState currState = Governor.getRobotState();
        if(DriverStation.isDisabled()){
            candle.animate(new LarsonAnimation(0xFF, 0x10, 0x0, 0, 0.4, LED_COUNT, BounceMode.Back, 8, 8), 0);
            clearAnimationFlag = true;
            // setColor(new Color(0xFF, 0x10, 0x0));
        }
        else if(DriverStation.isAutonomous()){
            candle.animate(new RainbowAnimation(0.8, 0.4, LED_COUNT, false, 0), 0);
            // setColor(new Color(0, 0, 0));
            clearAnimationFlag = true;
        }
        else{
            if(clearAnimationFlag){
                candle.clearAnimation(0);
                clearAnimationFlag = false;
            }
            
            if(currState != lastState){
                switch (currState) {
                    case NEUTRAL:
                        if(RobotContainer.sensors.getShooterSensor()) {
                            if(RobotContainer.sensors.getLoaderSensor()){
                                setColor(new Color(0, 255, 0));
                            }
                            else{
                                flash(new Color(87, 255, 139));
                            }
                            
                        }
                        else setColor(new Color(255, 255, 255));
                        break;
                    case INTAKE:
                        setColor(new Color(255, 0, 255));
                        break;
                    case SOURCE:
                        setColor(new Color(255, 105, 180));
                        break;
                    case SHOOT_PREP:
                        setColor(new Color(255, 255, 0));
                        break;
                    case SHOOT:
                        setColor(new Color(0, 255, 0));
                        break;
                    case AMP:
                        setColor(new Color(0, 255, 255));
                        break;
                    case AMP_ADJ:
                        setColor(new Color(0, 255, 255));
                        break;
                    case MISC:
                        setColor(new Color(0x9, 0x22, 0x15));
                        break;
                    default:
                        setColor(new Color(0, 0, 0));
                        break;
                }

                lastState = currState;
            }
        }
        
    }

    public void setColor(Color color){
        candle.setLEDs(color.r, color.g, color.b);
        clearAnimationFlag = true;
    }

    public void flash(Color color){
        candle.animate(new StrobeAnimation(color.r, color.g, color.b, 0, 0.2, LED_COUNT-8, 8));
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
