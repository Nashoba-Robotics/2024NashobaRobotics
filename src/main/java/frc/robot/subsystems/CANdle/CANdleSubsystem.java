package frc.robot.subsystems.CANdle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.Governor;
import frc.robot.Governor.RobotState;

public class CANdleSubsystem {

    static RobotState state = RobotState.UNKNOWN;

    Animation neutral = new StrobeAnimation(51, 43, 43, 75, 0, 68, 8);     //Dark Grey
    Animation zero = new StrobeAnimation(255, 255, 255, 255, 0, 68, 8);        //White
    Animation unknown = new StrobeAnimation(58, 176, 164, 75, 0, 68, 8);     //Cyan
    Animation misc = new StrobeAnimation(176, 153, 153, 75, 0, 68, 8);        //Light Grey
        
    Animation transition = new StrobeAnimation(0, 0, 0, 0, 0, 68, 8);  //off
        
    Animation intake = new StrobeAnimation(0, 0, 0, 75, 0, 68, 8);      //Bright Green
    Animation source = new StrobeAnimation(0, 0, 0, 75, 0, 68, 8);      //Blue
    Animation shoot_prep = new StrobeAnimation(0, 0, 0, 75, 0, 68, 8);  //Bright Orange
    Animation shoot = new StrobeAnimation(0, 0, 0, 75, 0, 68, 8);       //Bright Red
    Animation amp = new StrobeAnimation(0, 0, 0, 75, 0, 68, 8);         //Purple
    Animation amp_adj = new StrobeAnimation(0, 0, 0, 75, 0, 68, 8);     //Pink

    
    Animation rightFrontTeleopAnimation = new StrobeAnimation(46, 133, 23, 75, 0, 10, 8);
    Animation leftFrontTeleopAnimation = new StrobeAnimation(46, 133, 23, 75, 0, 17, 59);
    Animation rightBackTeleopAnimation = new StrobeAnimation(46, 133, 23, 75, 0, 24, 18);
    Animation LeftBackTeleopAnimation = new StrobeAnimation(46, 133, 23, 75, 0, 26, 43);
        

    public CANdleSubsystem(){
        CANdle candle = new CANdle(0);
        
        Animation autoDefualt = new RainbowAnimation(1, 0.5, 68, false, 8);
        Animation disabledDefualt = new LarsonAnimation(237, 77, 19, 75, 0.75, 68, BounceMode.Center, 4, 8);
        
        
    }

    public Animation getStateAnimation(){
        Animation returnAnimation = new StrobeAnimation(0, 0, 0);
        // String stateName = Governor.getRobotState().toString();
        // return this.stateName
        if(Governor.getRobotState() == RobotState.NEUTRAL){
            returnAnimation = this.neutral;
        }
        else if(Governor.getRobotState() == RobotState.ZERO){
            returnAnimation = this.zero;
        }
        else if(Governor.getRobotState() == RobotState.UNKNOWN){
            returnAnimation = this.unknown;
        }        
        else if(Governor.getRobotState() == RobotState.MISC){
            returnAnimation = this.misc;
        }        
        else if(Governor.getRobotState() == RobotState.TRANSITION){
            returnAnimation = this.transition;
        }
        else if(Governor.getRobotState() == RobotState.INTAKE){
            returnAnimation = this.intake;
        }                
        else if(Governor.getRobotState() == RobotState.SOURCE){
            returnAnimation = this.source;
        }                
        else if(Governor.getRobotState() == RobotState.SHOOT_PREP){
            returnAnimation = this.shoot_prep;
        }                
        else if(Governor.getRobotState() == RobotState.SHOOT){
            returnAnimation = this.shoot;
        }                
        else if(Governor.getRobotState() == RobotState.AMP){
            returnAnimation = this.amp;
        }                
        else if(Governor.getRobotState() == RobotState.AMP_ADJ){
            returnAnimation = this.amp_adj;
        }                
        return returnAnimation;
    }

}
