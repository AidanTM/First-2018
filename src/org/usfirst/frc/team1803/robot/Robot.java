package org.usfirst.frc.team1803.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	static RobotDrive leftMotors;
	static RobotDrive rightMotors;
	static RobotDrive gripperMotors;
	static Joystick playerController;
	static DriverStation ds;
	
	static final double motorOffset = 0.92;
	
	double speedMultiplier;
	double[] stickVel;
	
	static int debugLoop;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		leftMotors = new RobotDrive(1, 2);
		rightMotors = new RobotDrive(3, 4);
		gripperMotors = new RobotDrive(5, 6);
		playerController = new Joystick(0);
		ds = DriverStation.getInstance();
		
		leftMotors.setSafetyEnabled(true);
		rightMotors.setSafetyEnabled(true);
		
		leftMotors.setExpiration(0.1);
		rightMotors.setExpiration(0.1);
		
		speedMultiplier = 1.0;
		stickVel = new double[6];
		
		debugLoop = 0;
		
		System.out.println("Robot Init Done");
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic()
	{
		//Check to see if the controller wants to slow the robot down.
		if (playerController.getRawButton(6) || playerController.getRawButton(9)) speedMultiplier = 1.0;
		else speedMultiplier = 0.5;
		
		//Set the stick values based on the sticks and speedMultiplier.
		stickVel[0] = (double)Math.round(playerController.getRawAxis(1) * speedMultiplier * 100d) / 100d;
		if (playerController.getRawAxis(5) < 0) stickVel[1] = (double)Math.round(playerController.getRawAxis(5) * speedMultiplier * 100d) / 100d * motorOffset;
		else stickVel[1] = (double)Math.round(playerController.getRawAxis(5) * speedMultiplier * 100d) / 100d;//To equalize the offset in motor speed.
		stickVel[2] = Math.round(playerController.getRawAxis(2) * 100d) / 100d;
		stickVel[3] = Math.round(playerController.getRawAxis(0) * speedMultiplier * 100d) / 100d;
		stickVel[5] = Math.round((1d - Math.abs(stickVel[0] / speedMultiplier)) * 100d) / 100d;
		//Run the motors.
		if (playerController.getRawButton(5))
		{
			leftMotors.tankDrive(-stickVel[0],stickVel[0],true);
			rightMotors.tankDrive(stickVel[1],-stickVel[1],true);
		}
		else
		{
			leftMotors.tankDrive(-stickVel[0] + (stickVel[3] * stickVel[5]), stickVel[0] - (stickVel[3] * stickVel[5]),true);
			rightMotors.tankDrive((stickVel[0] + (stickVel[3] * stickVel[5])) * motorOffset, (-stickVel[0] - (stickVel[3] * stickVel[5])) * motorOffset,true);
		}
		if (playerController.getRawButton(1) && stickVel[2] > 0.1) gripperMotors.tankDrive(-stickVel[2], -stickVel[2]);
		else if (stickVel[2] > 0.1) gripperMotors.tankDrive(stickVel[2], stickVel[2]);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic()
	{
		Jaguar test = new Jaguar(5);
		LiveWindow.addActuator("Left Fly Wheel", 0, test);
		teleopPeriodic();
		
		if (debugLoop % 1000 == 0)
		{
			printWarn("stickVel: " + (stickVel[3] + " " + stickVel[5]));
			debugLoop = 0;
		}
		else debugLoop++;
	}
	
	public static void printWarn(String msg)
	{
		DriverStation.reportWarning(msg, false);
	}
}

