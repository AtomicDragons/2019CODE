/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot;

import frc.LoadMon.CasseroleRIOLoadMonitor;
import java.rmi.UnexpectedException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Ultrasonic;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  CasseroleRIOLoadMonitor loadMon;
  
  static final int BAUD_RATE = 115200;
	
	SerialPort visionPort = null;
  int loopCount = 0;
  
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  WPI_TalonSRX frontLeft;
	WPI_VictorSPX backRight;
	WPI_TalonSRX frontRight;
  WPI_TalonSRX backLeft;
  WPI_TalonSRX wheel;
  WPI_TalonSRX arm;

  JeVoisInterface cam;

  UsbCamera visionCam;
  MjpegServer camServer;
  
	SpeedControllerGroup leftMotors;
	SpeedControllerGroup rightMotors;
	
  DifferentialDrive drive;
	
  Joystick driveStick;
  Joystick opStick;

  //SerialPort ultra;
  AnalogInput ultraRight;
  AnalogInput ultraLeft;
  
  double ultraRightVoltage;
  double ultraRightDistance;

  double ultraLeftVoltage;
  double ultraLeftDistance;
  
  Compressor compressor;
  DoubleSolenoid frontSully;
  DoubleSolenoid backSully;

  Relay relay;

  Encoder armEncoder;

  ScheduledExecutorService scheduler;

  final double PULSE_CONVERSION = .27692308;
  double SPEED_MULTIPLIER = .6;

  @Override
  public void robotInit() {

    CameraServer.getInstance().startAutomaticCapture();
    
    cam = new JeVoisInterface(true);
    loadMon = new CasseroleRIOLoadMonitor();

   

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    RobotMap map = new RobotMap();

		backRight = new WPI_VictorSPX(map.FRONT_RIGHT_MOTOR_PORT);
    frontLeft = new WPI_TalonSRX(map.FRONT_LEFT_MOTOR_PORT);
		frontRight = new WPI_TalonSRX(map.BACK_RIGHT_MOTOR_PORT);
    backLeft = new WPI_TalonSRX(map.BACK_LEFT_MOTOR_PORT);
    wheel = new WPI_TalonSRX(map.SCISSOR_WHEEL_MOTOR_PORT);
    arm = new WPI_TalonSRX(map.ARM_MOTOR_PORT);
  

		leftMotors = new SpeedControllerGroup (frontLeft, backLeft);
		backRight.setInverted(true);
		rightMotors = new SpeedControllerGroup (backRight, frontRight);
		
		drive = new DifferentialDrive (leftMotors, rightMotors);
		drive.setSafetyEnabled(false);

    driveStick = new Joystick(map.DRIVESTICK_PORT);
    opStick = new Joystick(map.OPSTICK_PORT);

    ultraRight = new AnalogInput(map.ULTRA_RIGHT_PORT);
    ultraLeft = new AnalogInput(map.ULTRA_LEFT_PORT);

    compressor = new Compressor(0);
    frontSully = new DoubleSolenoid(map.FRONT_SULLY_PORT, map.FRONT_SULLY_PORT + 1);
    backSully = new DoubleSolenoid(map.BACK_SULLY_PORT, map.BACK_SULLY_PORT + 1);

    relay = new Relay(3);
    relay.set(Relay.Value.kForward);

    //armEncoder = new Encoder(0, 1, false,Encoder.EncodingType.k4X);
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }
  @Override
  public void disabledPeriodic() {
      System.out.print("s");
      System.out.println(cam.getPacketRxRate_PPS());
      System.out.print("r");
      System.out.println(loadMon.getCPULoadPct());

  }
  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }
  @Override
  public void teleopInit() {
    
    if (visionPort == null) return;
    System.out.println("pinging JeVois");
    String cmd = "ping";
    int bytes = visionPort.writeString(cmd);
    System.out.println("wrote " +  bytes + "/" + cmd.length() + " bytes, cmd: " + cmd);
    
}
  
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    /*
    if (visionPort == null) return;
    if (visionPort.getBytesReceived() > 0) {
        System.out.println("Waited: " + loopCount + " loops, Rcv'd: " + visionPort.readString());
        loopCount = 0;
      }
    else {
      ++loopCount;
    }
    */
    
    //getting data from ultrasonics and converting to distance
    ultraRightVoltage = ultraRight.getVoltage();
    ultraLeftVoltage = ultraLeft.getVoltage();
    ultraLeftDistance = (ultraLeftVoltage * 41.6667) - .8333;
    ultraRightDistance = (ultraRightVoltage * 41.6667) - .8333;
   
    compressor.setClosedLoopControl(true);
  
    SmartDashboard.putNumber("ultraRight Voltage",ultraRightVoltage);
    SmartDashboard.putNumber("ultraRight Distance",ultraRightDistance);
  
    SmartDashboard.putNumber("ultraLeft Voltage",ultraLeftVoltage);
    SmartDashboard.putNumber("ultraLeft Distance",ultraLeftDistance);

    if (driveStick.getRawButton(1)) {


      if (ultraLeftDistance > ultraRightDistance){
        drive.tankDrive(1 * 0.5f, ultraRightDistance / ultraLeftDistance * 0.5f);
      }
      else{
        drive.tankDrive(ultraLeftDistance / ultraRightDistance * 0.5f, 1 * 0.5f);
      }
    }
    else{
      drive.tankDrive(-1 * driveStick.getRawAxis(1) * 0.5f, -1 * driveStick.getRawAxis(5) * 0.5f, true);
    }
    

    
    if (driveStick.getRawButton(2)){
      SPEED_MULTIPLIER = 1;
    }
    else {
      SPEED_MULTIPLIER = .6;
    }

    
    if (driveStick.getRawAxis(4) > .1){ 
      if (driveStick.getRawAxis(0) > .1){
        drive.tankDrive(driveStick.getRawAxis(4) * SPEED_MULTIPLIER, driveStick.getRawAxis(4)*(1-driveStick.getRawAxis(0)) * SPEED_MULTIPLIER);
      }
      else if (driveStick.getRawAxis(0) < -.1) {
        drive.tankDrive (driveStick.getRawAxis(4) * (1+driveStick.getRawAxis(0)) * SPEED_MULTIPLIER, driveStick.getRawAxis(4) * SPEED_MULTIPLIER);
      }
      else{
        drive.tankDrive(1,1);
      }

    }
    else if (driveStick.getRawAxis(3) > .1) {
      if (driveStick.getRawAxis(0) > .1){
        drive.tankDrive(-driveStick.getRawAxis(3) * SPEED_MULTIPLIER, -driveStick.getRawAxis(3)*(1-driveStick.getRawAxis(0)) * SPEED_MULTIPLIER);
    }
    else if (driveStick.getRawAxis (0) < -.1) {
      drive.tankDrive(-driveStick.getRawAxis(3) * (1+driveStick.getRawAxis(0)) * SPEED_MULTIPLIER, -driveStick.getRawAxis(3) * SPEED_MULTIPLIER);
    }
    else {
      drive.tankDrive(-1,-1);
    }
  }
    else{
      drive.tankDrive(0, 0);
    }

    //drive.tankDrive(1, 1);

    //controls the back scissor lift wheel  motor 
    if (opStick.getRawButton(1)){
      wheel.set(1);
    }
    else {
      wheel.set(0);
    }
 

    //pneumatics
    if(opStick.getRawButton(9)){
      frontSully.set(DoubleSolenoid.Value.kForward);
      backSully.set(DoubleSolenoid.Value.kForward);
    }
    else if (opStick.getRawButton(11)){
      frontSully.set(DoubleSolenoid.Value.kReverse);
    }
    else if (opStick.getRawButton(10)){
      backSully.set(DoubleSolenoid.Value.kReverse);
    }

  
    if(opStick.getRawButton(2)){
      arm.set(1);
      /*
      //for if we use timer which will prolly not happen
      scheduler = Executors.newScheduledThreadPool(1);
      scheduler.scheduleAtFixedRate(()->{arm.set(0);}, 0, 1 * 1000, TimeUnit.MILLISECONDS);
      */
    }
    
    System.out.println("==============+++==============");
    System.out.print("Vision Online: ");
    System.out.println(cam.isVisionOnline());
    System.out.print("Target Visible: ");
    System.out.println(cam.isTgtVisible());
    System.out.print("Target Angle: ");
    System.out.println(cam.getTgtAngle_Deg());
    System.out.print("Target Range:");
    System.out.println(cam.getTgtRange_in());
    System.out.print("Serial Packet RX Rate: ");
    System.out.println(cam.getPacketRxRate_PPS());
    System.out.print("JeVois Framerate: ");
    System.out.println(cam.getJeVoisFramerate_FPS());
    System.out.print("JeVois CPU Load: ");
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }
}
