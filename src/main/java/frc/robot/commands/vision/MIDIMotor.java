package frc.robot.commands.vision;
import java.util.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
//json imports
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.FileReader;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Path;


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
    
    private Map<String, CANSparkMax> noteMotors;

    private CANSparkMax motor;
    //Class Constructor
    public MIDIMotor(DriveSubsystem driveSubsystem){
        m_driveSubsystem = driveSubsystem;
        noteMotors = new HashMap<>();
        noteMotors.put("motor1", new CANSparkMax(DriveConstants.kFrontRightDrivingCanId, MotorType.kBrushless));  // Replace with actual CAN ID
        noteMotors.put("motor2", new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless));  // Replace with actual CAN ID
        noteMotors.put("motor3", new CANSparkMax(DriveConstants.kFrontLeftDrivingCanId, MotorType.kBrushless));  // Replace with actual CAN ID
        noteMotors.put("motor4", new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless));  // Replace with actual CAN ID
        noteMotors.put("motor5", new CANSparkMax(DriveConstants.kRearLeftDrivingCanId, MotorType.kBrushless));  // Replace with actual CAN ID
        noteMotors.put("motor6", new CANSparkMax(DriveConstants.kRearLeftTurningCanId, MotorType.kBrushless));  // Replace with actual CAN ID
        noteMotors.put("motor7", new CANSparkMax(DriveConstants.kRearRightDrivingCanId, MotorType.kBrushless));  // Replace with actual CAN ID
        noteMotors.put("motor8", new CANSparkMax(DriveConstants.kRearRightTurningCanId, MotorType.kBrushless));  // Replace with actual CAN ID
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

       try {
        // parsing file "your_data.json"
        Object obj = new JSONParser().parse(new FileReader("src/main/deploy/midi/grouped_notes_with_speed.json"));

        // typecasting obj to JSONArray
        JSONArray notesArray = (JSONArray) obj;

        // iterating over notesArray
        Iterator<?> iterator = notesArray.iterator();
        while (iterator.hasNext()) {
            double ammountOfInUseMotors = 0;
            JSONObject noteEntry = (JSONObject) iterator.next();

            // Getting Start Time
            double startTime = (double) noteEntry.get("Start Time");
            System.out.println("Start Time: " + startTime);

            // Getting notes array
            JSONArray notesArrayData = (JSONArray) noteEntry.get("0");

            // iterating over notesArrayData
            Iterator<?> dataIterator = notesArrayData.iterator();
            while (dataIterator.hasNext()) {
                ammountOfInUseMotors++;
                JSONObject noteData = (JSONObject) dataIterator.next();
                String noteType = (String) noteData.get("Note");
                double speed = (double) noteData.get("Speed");

                // Use startTime, noteType, and speed as needed
                System.out.println("Note: " + noteType + ", Speed: " + speed);
                int motorIndex = (int) ammountOfInUseMotors - 1;
                if (speed >= elapsedTime && speed <= elapsedTime +.25 && ammountOfInUseMotors != 9){
                    CANSparkMax assignedMotor = noteMotors.get("motor" + motorIndex);
                    if (assignedMotor != null) {
                        assignedMotor.set(speed);
                    }
                }
            }
        }
    } catch (Exception e) {
        e.printStackTrace();
        System.err.println("Error reading or parsing the JSON file.");
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