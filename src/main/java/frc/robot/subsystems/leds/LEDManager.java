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
import frc.robot.subsystems.sensors.SensorManager;

public class LEDManager extends SubsystemBase{
    private static CANdle candle;
    private RobotState lastState;

    private static final int LED_COUNT = 85;

    private boolean clearAnimationFlag = false;

    private Color red = new Color(255, 0, 0);
    private Color green = new Color(0, 255, 0);
    private Color blue = new Color(0, 0, 255);
    private Color disabled = new Color(0xFF, 0x10, 0x0);
    private Color intake = new Color(255, 0, 255);
    private Color amp = blue;
    private Color ampPrep = new Color(0, 255, 255);
    private Color source = new Color(255, 105, 180);
    private Color shootPrep = new Color(255, 255, 0);
    private Color shoot = green;
    private Color misc = new Color(0x9, 0x22, 0x15);
    private Color climb;

    //26 LEds on last strip
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
        if(DriverStation.isDisabled()){
            //TODO: Add check to see if the pivot motor was initizlied correctly
            candle.animate(new LarsonAnimation(0xFF, 0x10, 0x0, 0, 0.4, LED_COUNT, BounceMode.Back, 8, 8), 0);
            // candle.clearAnimation(0);

            // setColor(new Color(0, 0, 0), 8, LED_COUNT-8);

            //Check if we have a note
            if(RobotContainer.sensors.getShooterSensor())
                setColor(green, 4, 4);
            else
                setColor(red, 4, 4);

            // Check if the arm is "zeroed"
            if(RobotContainer.arm.getArmPivotAngle().getRadians() <= -0.87)
                setColor(green, 0, 4);
            else
                setColor(red, 0, 4);
            clearAnimationFlag = true;
        }
        else if(DriverStation.isAutonomous()){
            // candle.animate(new RainbowAnimation(0.8, 0.6, LED_COUNT-26-8, false, 8), 0);
            if(RobotContainer.sensors.getShooterSensor() || RobotContainer.sensors.getLoaderSensor())
                setColor(green);
            else
                setColor(red);

            clearAnimationFlag = true;
        }
        else{
            if(clearAnimationFlag){
                candle.clearAnimation(0);
                clearAnimationFlag = false;
            }
            RobotState currState = Governor.getDesiredRobotState();
            if(currState != lastState){
                switch (currState) {
                    case NEUTRAL:
                        if(RobotContainer.sensors.getShooterSensor() || RobotContainer.sensors.getIntakeSensor()){
                            if(RobotContainer.sensors.getLoaderSensor()){
                                setColor(new Color(0, 255, 0));
                            }
                            else{
                                setColor(new Color(255, 0, 0));
                            }

                        } 
                        else setColor(new Color(255, 255, 255));
                        break;
                    case INTAKE:
                        if(RobotContainer.sensors.getIntakeSensor() || RobotContainer.sensors.getLoaderSensor()) setColor(green);
                        else setColor(intake);
                        break;
                    case SOURCE:
                        setColor(source);
                        break;
                    case SHOOT_PREP:
                        setColor(shootPrep);
                        break;
                    case SHOOT:
                        setColor(shoot);
                        break;
                    case AMP:
                        setColor(amp);
                        break;
                    case AMP_ADJ:
                        setColor(ampPrep);
                        break;
                    case MISC:
                        setColor(misc);
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
        candle.setLEDs(color.r, color.g, color.b, 0, 8, LED_COUNT-8);
    }
    public void setColor(Color color, int startId, int length){
        candle.setLEDs(color.r, color.g, color.b, 0, startId, length);
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
