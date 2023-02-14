// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  WPI_VictorSPX driveLeftFrontVictor = new WPI_VictorSPX(3);
  WPI_VictorSPX driveRightFrontVictor = new WPI_VictorSPX(1);
  WPI_VictorSPX driveRightBackVictor = new WPI_VictorSPX(2);
  WPI_VictorSPX driveLeftBackVictor = new WPI_VictorSPX(4);
  MotorControllerGroup driveLeft = new MotorControllerGroup(driveLeftBackVictor, driveLeftFrontVictor);
  MotorControllerGroup driveRight = new MotorControllerGroup(driveRightBackVictor, driveRightFrontVictor);
  DifferentialDrive m_drive = new DifferentialDrive(driveLeft, driveRight);
  UsbCamera usbcamera = new UsbCamera("Camera 0", 0);

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);

  ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  //Joystick j = new Joystick(0);
  XboxController XboxController = new XboxController(0);

  /*
   * Magic numbers. Use these to adjust settings.
  
   */
  SlewRateLimiter filter = new SlewRateLimiter(0.35);
  PIDController pid = new PIDController(kDefaultPeriod, kDefaultPeriod, kDefaultPeriod);
  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 1.0;

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.375;

  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 6.0;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -0.25;
  

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  
    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftBackVictor.setInverted(false);
    driveLeftFrontVictor.setInverted(false);
    driveRightBackVictor.setInverted(false);
    driveRightFrontVictor.setInverted(false);

    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    arm.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    driveLeftBackVictor.set(ControlMode.PercentOutput, left);
    driveLeftFrontVictor.set(ControlMode.PercentOutput, left);
    driveRightBackVictor.set(ControlMode.PercentOutput, right);
    driveRightFrontVictor.set(ControlMode.PercentOutput, right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    arm.set(percent);
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("X-Axis Acceleration", m_gyro.getAccelX());
    SmartDashboard.putNumber("X-Axis Angle", m_gyro.getYComplementaryAngle());

  }
  
  
  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {
    driveLeftBackVictor.setNeutralMode(NeutralMode.Brake);
    driveLeftFrontVictor.setNeutralMode(NeutralMode.Brake);
    driveRightBackVictor.setNeutralMode(NeutralMode.Brake);
    driveRightFrontVictor.setNeutralMode(NeutralMode.Brake);

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    if (m_autoSelected == kConeAuto) {
      autonomousIntakePower = INTAKE_OUTPUT_POWER;
    } else if (m_autoSelected == kCubeAuto) {
      autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    if (m_autoSelected == kNothingAuto) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
      return;
    }

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    if (timeElapsed < ARM_EXTEND_TIME_S) {
      setArmMotor(ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
      setArmMotor(0.0);
      setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
      setArmMotor(-ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    driveLeftBackVictor.setNeutralMode(NeutralMode.Coast);
    driveLeftFrontVictor.setNeutralMode(NeutralMode.Coast);
    driveRightBackVictor.setNeutralMode(NeutralMode.Coast);
    driveRightFrontVictor.setNeutralMode(NeutralMode.Coast);

    lastGamePiece = NOTHING; 
    m_gyro.reset();
  }

  @Override
  public void teleopPeriodic() {

    getlimelightcontrols();

    double armPower;
    if (XboxController.getRawButton(7)) {
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
    } else if (XboxController.getRawButton(5)) {
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    double balancePower = m_gyro.getYComplementaryAngle()*.055;
    
    setArmMotor(armPower);

    if(XboxController.getXButton()) {
      if (m_gyro.getYComplementaryAngle() > .5) {
        driveLeftBackVictor.set(balancePower);
        driveRightBackVictor.set(-balancePower);
        driveLeftFrontVictor.set(balancePower);
        driveRightFrontVictor.set(-balancePower);
      }
      else if (m_gyro.getYComplementaryAngle() < -.5){
        driveLeftBackVictor.set(balancePower);
        driveRightBackVictor.set(-balancePower);
        driveLeftFrontVictor.set(balancePower);
        driveRightFrontVictor.set(-balancePower);


      }
    }
    double intakePower;
    int intakeAmps;
    if (XboxController.getRawButton(8)) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (XboxController.getRawButton(6)) {
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);
    

    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */
    //setDriveMotors(-j.getRawAxis(1), -j.getRawAxis(2));
    //m_drive.curvatureDrive(filter.calculate(XboxController.getRightX()), -XboxController.getLeftY(), true);

    double turnmultiplier = 1;
    if (XboxController.getRightBumper()) {
      turnmultiplier = .25;
    } 
    m_drive.curvatureDrive((turnmultiplier * XboxController.getRightX()), XboxController.getLeftY(), true);
      //m_drive.tankDrive(-XboxController.getLeftY(), XboxController.getRightY(), true);
  }
public void getlimelightcontrols() {

  
  double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);  
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  SmartDashboard.putNumber("Target X", tx);
  SmartDashboard.putNumber("Target y", ty);
  SmartDashboard.putNumber("Target Area", ta);
  SmartDashboard.putNumber("Target Present", tv);
  double locate = tx*.03;
  double travelTo = (13-ta)*.08;
  if (tv == 1) {
    if (XboxController.getAButton()) {
      if (travelTo > .3) {
        m_drive.arcadeDrive(locate, -.3);
      } else {
      m_drive.arcadeDrive(locate, -travelTo);
      }
    // if (XboxController.getBButton()) {
    //   driveLeftBackVictor.set(locate);
    //   driveRightBackVictor.set(locate);
    //   driveLeftFrontVictor.set(locate);
    //   driveRightFrontVictor.set(locate);
    // }
    // if (XboxController.getAButton()) {

    //   if (find > .2){
    //     driveLeftBackVictor.set(.2);
    //     driveRightBackVictor.set(-.21);
    //     driveLeftFrontVictor.set(.2);
    //   driveRightFrontVictor.set(-.21);
    //   } else {
    //   driveLeftBackVictor.set(travelTo);
    //   driveRightBackVictor.set(-travelTo);
    //   driveLeftFrontVictor.set(travelTo);
    //   driveRightFrontVictor.set(-travelTo);
    //   }
    // }
    }
  }
}
}
