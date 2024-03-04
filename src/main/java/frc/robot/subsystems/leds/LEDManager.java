package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;

public class LEDManager extends SubsystemBase{
    private static CANdle candle;
    private RobotState lastState;

    private static final int LED_COUNT = 83;

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
        if(DriverStation.isAutonomous()){
            candle.animate(new RainbowAnimation(0.8, 0.4, LED_COUNT, false, 0), 0);
            // setColor(new Color(0, 0, 0));
            clearAnimationFlag = true;
        }
        else if(DriverStation.isDisabled()){
            candle.animate(new LarsonAnimation(0xFF, 0x10, 0x0, 0, 0.4, LED_COUNT, BounceMode.Back, 8), 0);
            clearAnimationFlag = true;
            // setColor(new Color(0xFF, 0x10, 0x0));
        }
        else{
            if(clearAnimationFlag){
                candle.clearAnimation(0);
                clearAnimationFlag = false;
            }
            RobotState currState = Governor.getRobotState();
            if(currState != lastState){
                switch (currState) {
                    case NEUTRAL:
                        if(RobotContainer.loader.getShooterSensor()) setColor(new Color(255, 255, 255));
                        else setColor(new Color(255, 0, 0));
                        break;
                    case INTAKE:
                        setColor(new Color(255, 0, 255));
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
