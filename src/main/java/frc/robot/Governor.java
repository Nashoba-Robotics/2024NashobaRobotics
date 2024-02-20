package frc.robot;

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
    
    public enum RobotState {
        NEUTRAL,
        ZERO, //?

        UNKNOWN,
        MISC,

        TRANSITION,

        INTAKE,
        SOURCE,
        SHOOT_PREP,
        SHOOT,
        AMP_ADJ,
        AMP
    }

    public static void setRobotState(RobotState robotState) {
        if(state != RobotState.TRANSITION) {
            state = RobotState.TRANSITION;
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
                case AMP_ADJ:
                    toAmpAdj();
                    break;
                case AMP:
                    toAmp();
                    break;
            }
        }
    }

    public static RobotState getRobotState() {
        return state;
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
    private static void toAmpAdj() {
        CommandScheduler.getInstance().schedule(new ToAmpAdj());
    }
    private static void toAmp() {
        CommandScheduler.getInstance().schedule(new ToAmp());
    }

    public static Command getSetStateCommand(RobotState state) {
        return new InstantCommand(() -> Governor.state = state);
    }


}