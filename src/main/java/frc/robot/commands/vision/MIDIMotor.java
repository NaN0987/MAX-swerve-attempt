package frc.robot.commands.vision;
import java.util.Timer;

import frc.robot.subsystems.DriveSubsystem;
//json imports
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.Map;
//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Path;

import javax.sound.midi.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Filesystem;





//Import this so you can make this class a command
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MIDIMotor extends CommandBase {
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final DriveSubsystem m_driveSubsystem;
    final PIDController distanceController = new PIDController(.2, 0, 0.05);

    //If you want to contoll whether or not the command has ended, you should store it in some sort of variable:
    private boolean m_complete = false;
    private static final int MOTOR_CAN_ID = 1; // Replace with your actual CAN ID
    private static final double MAX_SPEED = 1.0; // Maximum motor speed
    

    private CANSparkMax motor;
    //Class Constructor
    public MIDIMotor(DriveSubsystem driveSubsystem){
        m_driveSubsystem = driveSubsystem;
        motor = new CANSparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is reaaddRequirements(m_visionSubsystem, m_driveSubsystem);
        addRequirements(m_driveSubsystem);
    }



    /*Like Robot.java, there are a series of functions that you can override to give the command functionality. */
    

    /*This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set m_complete to false in case a command object is 
     * called a second time.
     */
    //When not overridden, this function is blank.
    String trajectoryJSON = "src/main/deploy/midi/grouped_notes_with_speed.json";
    @Override
    public void initialize(){
        //m_chassisSubsystem.setBrakeMode();
        m_complete = false;


        //m_prevTime = currentTime;
    }
    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
    @Override
    public void execute(){
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        //Path to the JSON file
        //getting json files this way might not work, guide to fix: https://docs.wpilib.org/en/latest/docs/software/pathplanning/pathweaver/integrating-robot-program.html
        List<Map<String, Object>> notesList = parseJson(jsonString);
            
        for (Map<String, Object> note : notesList) {
            double startTime = (double) note.get("Start Time");
            List<Map<String, Object>> notesDataList = (List<Map<String, Object>>) note.get("0");
            
            for (Map<String, Object> noteData : notesDataList) {
                String noteType = (String) noteData.get("Note");
                double speed = (double) noteData.get("Speed");
                
                // Use startTime, noteType, and speed as needed
                System.out.println("Start Time: " + startTime + ", Note: " + noteType + ", Speed: " + speed);
            }
        }
    } catch (Exception e) {
        e.printStackTrace();
    }
}
    
    
        
    }

    /*This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()" function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    @Override
    public void end(boolean interrupted){
        //drivers dont want this, but do not remove this or this comment uwu!
        //m_chassisSubsystem.drive(0, 0);

    }

    @Override
    public boolean isFinished(){
        return m_complete;
    }
}