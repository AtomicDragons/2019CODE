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


  Boolean armIsFront;
  Boolean armIsBack;
  Boolean oldArmIsBack;
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
  WPI_VictorSPX frontLeft;
	WPI_VictorSPX backRight;
	WPI_TalonSRX frontRight;
  WPI_TalonSRX backLeft;
  WPI_TalonSRX arm;
  WPI_TalonSRX ball;
  Spark wheel1;
  Spark wheel2;

  Spark scissorOne;
  Spark scissorTwo;

  JeVoisInterface cam;

  UsbCamera visionCam;
  MjpegServer camServer;
  
	SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;
  SpeedControllerGroup wheelMotors;
	
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
  DoubleSolenoid frontScissorSully;
  DoubleSolenoid backScissorSully;
  DoubleSolenoid wheelSully;
  DoubleSolenoid hatchSully;
  
  DigitalInput frontLimitSwitch;
  DigitalInput backLimitSwitch;
  
  Relay relay;

  Encoder armEncoder;
  static final int encoderNoMore = 1;
  static final int encoderBack = 2;
  static final int targetEncoder = 3;

  //used for hatch code later! woo_hoo! :)
  boolean hatchController = true;
  boolean hatchStatus = true;

  ScheduledExecutorService scheduler;

  final double PULSE_CONVERSION = .27692308;
  double SPEED_MULTIPLIER = .6;

  @Override
  public void robotInit() {

    armIsFront = true;
    armIsBack = false;

    CameraServer.getInstance().startAutomaticCapture();
    
    cam = new JeVoisInterface(true);
    loadMon = new CasseroleRIOLoadMonitor();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    RobotMap map = new RobotMap();

		backRight = new WPI_VictorSPX(map.FRONT_RIGHT_MOTOR_PORT);
    frontLeft = new WPI_VictorSPX(map.FRONT_LEFT_MOTOR_PORT);
		frontRight = new WPI_TalonSRX(map.BACK_RIGHT_MOTOR_PORT);
    backLeft = new WPI_TalonSRX(map.BACK_LEFT_MOTOR_PORT);
    wheel1 = new Spark(map.SCISSOR_WHEEL_ONE_MOTOR_PORT);
    wheel2 = new Spark(map.SCISSOR_WHEEL_TWO_MOTOR_PORT);
    arm = new WPI_TalonSRX(map.ARM_MOTOR_PORT);
    ball = new WPI_TalonSRX(map.SHOOTY_WHEEL_MOTOR_PORT);

		leftMotors = new SpeedControllerGroup (frontLeft, backLeft);
    frontRight.setInverted(false);
    backRight.setInverted(true);
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    rightMotors = new SpeedControllerGroup (backRight, frontRight);
    wheelMotors = new SpeedControllerGroup(wheel1, wheel2);
		
		drive = new DifferentialDrive (leftMotors, rightMotors);
		drive.setSafetyEnabled(false);

    driveStick = new Joystick(map.DRIVESTICK_PORT);
    opStick = new Joystick(map.OPSTICK_PORT);

    ultraRight = new AnalogInput(map.ULTRA_RIGHT_PORT);
    ultraLeft = new AnalogInput(map.ULTRA_LEFT_PORT);

    compressor = new Compressor(0);
    frontScissorSully = new DoubleSolenoid(0, map.FRONT_SULLY_PORT, map.FRONT_SULLY_PORT + 1);
    backScissorSully = new DoubleSolenoid(0, map.BACK_SULLY_PORT, map.BACK_SULLY_PORT + 1);
    hatchSully = new DoubleSolenoid(1, map.HATCH_SULLY_PORT, map.HATCH_SULLY_PORT + 1);
   
    relay = new Relay(3);
    relay.set(Relay.Value.kForward);

    /*
    armEncoder = new Encoder(0, 1, false,Encoder.EncodingType.k4X);
    */
    frontLimitSwitch = new DigitalInput(map.FRONT_LIMIT_SWITCH_PORT);
    backLimitSwitch = new DigitalInput(map.BACK_LIMIT_SWITCH_PORT);
  
    
  }

  @Override
  public void robotPeriodic() {
  }
  @Override
  public void disabledPeriodic() {
    //unused
     

  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

  }

  @Override
  public void autonomousPeriodic() {
    //unused
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
    //unused, for vision
    /*
    if (visionPort == null) return;
    System.out.println("pinging JeVois");
    String cmd = "ping";
    int bytes = visionPort.writeString(cmd);
    System.out.println("wrote " +  bytes + "/" + cmd.length() + " bytes, cmd: " + cmd);
    */
    
}
  
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
   
    //TODO: check if works in init
    compressor.setClosedLoopControl(true);
  
    SmartDashboard.putNumber("ultraRight Voltage",ultraRightVoltage);
    SmartDashboard.putNumber("ultraRight Distance",ultraRightDistance);
  
    SmartDashboard.putNumber("ultraLeft Voltage",ultraLeftVoltage);
    SmartDashboard.putNumber("ultraLeft Distance",ultraLeftDistance);

    //ultrasonic sensor movement
    if (driveStick.getRawButton(ControlsMap.DRIVE_STICK_ALLIGN)) {
      if (ultraLeftDistance > ultraRightDistance){
        drive.tankDrive(1 * 0.5f, ultraRightDistance / ultraLeftDistance * 0.5f);
      }
      else{
        drive.tankDrive(ultraLeftDistance / ultraRightDistance * 0.5f, 1 * 0.5f);
      }
    }
    else{
      drive.tankDrive(-1 * driveStick.getRawAxis(ControlsMap.DRIVE_STICK_ALLIGN) * 0.5f, -1 * driveStick.getRawAxis(5) * 0.5f, true);
    }
    

    //speed limiter
    if (driveStick.getRawButton(ControlsMap.DRIVE_STICK_CHANGE_SPEED_MULTIPLIER)){
      SPEED_MULTIPLIER = 1;
    }
    else {
      SPEED_MULTIPLIER = .6;
    }
    /*
    //GTA Drive (GTA = Grand Theft Auto, for dumb peoplelike MATEO PARRADO who don't know)
    if (driveStick.getRawAxis(ControlsMap.DRIVE_STICK_FORWARDS) > .1){ 
      // right
      if (driveStick.getRawAxis(ControlsMap.DRIVE_STICK_STEERING) > .1){
        drive.tankDrive(driveStick.getRawAxis(ControlsMap.DRIVE_STICK_FORWARDS) * SPEED_MULTIPLIER, driveStick.getRawAxis(ControlsMap.DRIVE_STICK_FORWARDS)*(1-driveStick.getRawAxis(ControlsMap.DRIVE_STICK_STEERING)) * SPEED_MULTIPLIER);
      }
      // left
      else if (driveStick.getRawAxis(ControlsMap.DRIVE_STICK_STEERING) < -.1) {
        drive.tankDrive (driveStick.getRawAxis(ControlsMap.DRIVE_STICK_FORWARDS) * (1+driveStick.getRawAxis(ControlsMap.DRIVE_STICK_STEERING)) * SPEED_MULTIPLIER, driveStick.getRawAxis(ControlsMap.DRIVE_STICK_FORWARDS) * SPEED_MULTIPLIER);
      }
      // straight
      else{
        drive.tankDrive(1,1);
      }
    }
    //backwards
    else if (driveStick.getRawAxis(ControlsMap.DRIVE_STICK_BACKWARDS) > .1) {
      // right
      if (driveStick.getRawAxis(ControlsMap.DRIVE_STICK_STEERING) > .1){
        drive.tankDrive(-driveStick.getRawAxis(ControlsMap.DRIVE_STICK_BACKWARDS) * SPEED_MULTIPLIER, -driveStick.getRawAxis(ControlsMap.DRIVE_STICK_BACKWARDS)*(1-driveStick.getRawAxis(ControlsMap.DRIVE_STICK_STEERING)) * SPEED_MULTIPLIER);
      }
      // left
      else if (driveStick.getRawAxis (0) < -.1) {
        drive.tankDrive(-driveStick.getRawAxis(ControlsMap.DRIVE_STICK_BACKWARDS) * (1+driveStick.getRawAxis(ControlsMap.DRIVE_STICK_STEERING)) * SPEED_MULTIPLIER, -driveStick.getRawAxis(ControlsMap.DRIVE_STICK_BACKWARDS) * SPEED_MULTIPLIER);
      }
      // straight
      else {
        drive.tankDrive(-1,-1);
      }
    }
    // break if no button is pressed
    else{
      drive.tankDrive(0, 0);
      
    }
    */
    if(driveStick.getRawAxis(4) > .1){
      drive.curvatureDrive(driveStick.getRawAxis(4), driveStick.getRawAxis(0), false);
    }
    else if(driveStick.getRawAxis(3) > .1){
      drive.curvatureDrive(-driveStick.getRawAxis(3), driveStick.getRawAxis(0), false);
    }
    else{
      drive.curvatureDrive(0, driveStick.getRawAxis(0), true);
    }
    //drive.tankDrive(driveStick.getRawAxis(1), driveStick.getRawAxis(5));
    //drive.tankDrive(1, 1);

    //controls the back scissor lift wheel  motor 
   
    if (opStick.getRawButton(ControlsMap.OP_STICK_MOVE_SCISSOR_WHEELS)){
      wheelMotors.set(1);
    }
    else {
      wheelMotors.set(0);
    }
    

    //if scissor lift is dropped, shoot both forwards
    if(opStick.getRawButton(ControlsMap.OP_STICK_LOWER_SCISSOR_LIFT)){
      frontScissorSully.set(DoubleSolenoid.Value.kForward);
      backScissorSully.set(DoubleSolenoid.Value.kForward);
    }//raise first piston
    else if (opStick.getRawButton(ControlsMap.OP_STICK_RAISE_FRONT_SCISSOR)){
      frontScissorSully.set(DoubleSolenoid.Value.kReverse);
    }//raise second piston
    else if (opStick.getRawButton(ControlsMap.OP_STICK_RAISE_BACK_SCISSOR)){
      backScissorSully.set(DoubleSolenoid.Value.kReverse);
    }
    
    // controls hatch moving in and out with just one button
    if (opStick.getRawButton(ControlsMap.OP_STICK_ARM_PISTON) && hatchStatus){
      hatchController = !hatchController;
      hatchStatus = false;
      if (hatchController == true){
        hatchSully.set(DoubleSolenoid.Value.kForward);
      }
      else if (hatchController == false){
        hatchSully.set(DoubleSolenoid.Value.kReverse);
      }
    }
    else if(!opStick.getRawButton(ControlsMap.OP_STICK_ARM_PISTON)){
      hatchStatus = true;
      hatchSully.set(DoubleSolenoid.Value.kOff);
    }

    //intake and release balls
    if(opStick.getRawButton(ControlsMap.OP_STICK_SHOOTY_WHEEL_INTAKE)){
      ball.set(1);
    }
    else if(opStick.getRawButton(ControlsMap.OP_STICK_SHOOTY_WHEEL_OUTTAKE)){
      ball.set(-1);

    }
    else{
      ball.set(0);
    }

  // move arm forwards or backwards
  if ((opStick.getRawAxis(ControlsMap.OP_STICK_ARM)) > .1 || ((opStick.getRawAxis(ControlsMap.OP_STICK_ARM)) < -0.1 )){
      arm.set(opStick.getRawAxis(ControlsMap.OP_STICK_ARM));
  }
  else{
    arm.set(0);
  }

  if(opStick.getRawButton(ControlsMap.OP_STICK_MOVE_SCISSOR_WHEELS)){
    scissorOne.set(1);
    scissorTwo.set(1);
  }
  /*
    if(opStick.getRawButton(ControlsMap.OP_STICK_MOVE_ARM_FOR_HATCH)){
      //BACKWARDS
      if(armIsBack){
        if(armEncoder.get() > -encoderNoMore){
          arm.set(-1);
        }
        else if(armEncoder.get() > -encoderBack){
          arm.set(0);
        }
        else{
          arm.set(0.5);
        }
      }
      //FORWARDS
      else{
        if(armEncoder.get() < encoderNoMore){
          arm.set(1);
        }
        else if(armEncoder.get() < encoderBack){
          arm.set(0);
        }
        else{
          arm.set(-0.5);
        }
      }
    }

    if(opStick.getRawButton(ControlsMap.OP_STICK_MOVE_ARM_FOR_BALL)){
      if(armIsFront){
        if(armEncoder.get() < encoderNoMore){
          arm.set(1);
        }
        else if(armEncoder.get() < encoderBack){
          arm.set(0);
        }
        else{
          arm.set(-0.5);
        }
      }
      else{
          if(armEncoder.get() < targetEncoder){
            arm.set(-0.2);
          }
          else{
            arm.set(0);
          }
        }
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
    //for if front limit switch is hit
    if(frontLimitSwitch.get()){
      armIsFront = true;
      armIsBack = false;
      armEncoder.reset();
    }

    */
    //for if back limit switch is hit

    /*
    if(backLimitSwitch.get()){
      armIsBack = true;
      armIsFront = false;
      armEncoder.reset();
    }
    */
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
  //sets arm to flat position
  public void resetArm(){
    if(armIsFront){
      if(armEncoder.get() < encoderBack){
        arm.set(0);
      }
      else{
        arm.set(-0.5);
      }
    }
    else{
      if(armEncoder.get() > -encoderBack){
        arm.set(0);
      }
      else{
        arm.set(0.5);
      }
    }
  }
}
