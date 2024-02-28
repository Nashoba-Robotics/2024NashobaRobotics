package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.setters.groups.ToAmp;
import frc.robot.commands.setters.groups.ToAmpAdj;
import frc.robot.commands.setters.groups.ToIntake;
import frc.robot.commands.setters.groups.ToNeutral;
import frc.robot.commands.setters.groups.ToShoot;
import frc.robot.commands.setters.groups.ToShootPrep;
import frc.robot.commands.setters.groups.ToSource;

public class Governor {
    private static RobotState state = RobotState.UNKNOWN;

    private static RobotState queuedState = RobotState.UNKNOWN;
    
    public enum RobotState {
        NEUTRAL,    //Wyoming
        ZERO, //?

        UNKNOWN,    //Ohio
        MISC,   //Florida

        TRANSITION,

        INTAKE, //Mississippi
        SOURCE, //Massachusetts
        SHOOT_PREP,
        SHOOT,  //Texas
        AMP, //California
        AMP_ADJ
    }

    public static void setRobotState(RobotState robotState) {
        setRobotState(robotState, false);
    }

    public static void setRobotState(RobotState robotState, boolean override) {
        if(state == RobotState.TRANSITION && !override) queuedState = robotState;
        if(robotState == RobotState.UNKNOWN || robotState == RobotState.MISC || robotState == RobotState.NEUTRAL) override = true;
        if(override || state != RobotState.TRANSITION) {
            if(robotState != RobotState.UNKNOWN && robotState != RobotState.MISC) state = RobotState.TRANSITION;
            else state = robotState;
            switch (robotState) {
                case NEUTRAL:
                    toNeutral();
                    break;
                case ZERO:
                    break;
                case UNKNOWN:
                    break;
                case MISC:
                    break;
                case TRANSITION:
                    System.out.println("How did I get here?");
                    break;
                case INTAKE:
                    toIntake();
                    break;
                case SOURCE:
                    toSource();
                    break;
                case SHOOT_PREP:
                    toShootPrep();
                    break;
                case SHOOT:
                    toShoot();
                    break;
                case AMP:
                    toAmp();
                    break;
                case AMP_ADJ:
                    toAmpAdj();
                    break;
            }
        }
    }

    public static RobotState getRobotState() {
        return state;
    }

    public static RobotState getQueuedState() {
        return queuedState;
    }

    public static void setQueuedState(RobotState queuedState) {
        Governor.queuedState = queuedState;
    }

    private static void toNeutral() {
        CommandScheduler.getInstance().schedule(new ToNeutral());
    }
    private static void toIntake() {
        CommandScheduler.getInstance().schedule(new ToIntake());
    }
    private static void toSource() {
        CommandScheduler.getInstance().schedule(new ToSource());
    }
    private static void toShootPrep() {
        CommandScheduler.getInstance().schedule(new ToShootPrep());
    }
    private static void toShoot() {
        CommandScheduler.getInstance().schedule(new ToShoot());
    }
    private static void toAmp() {
        CommandScheduler.getInstance().schedule(new ToAmp());
    }
    private static void toAmpAdj() {
        CommandScheduler.getInstance().schedule(new ToAmpAdj());
    }

    public static Command getSetStateCommand(RobotState state) {
        return new InstantCommand(() -> Governor.state = state);
    }


}