package frc.robot.subsystems.notedetection;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetector extends SubsystemBase{
    private static NoteDetectorIO io;
    private static NoteDetectorIOInputsAutoLogged inputs = new NoteDetectorIOInputsAutoLogged();

    public NoteDetector(){
        io = new NoteDetectorIOPhotonVision();
    }

    public static Pose2d[] getTargetPoses(){
        return inputs.notePos;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Note Detection", inputs);
    }
}
