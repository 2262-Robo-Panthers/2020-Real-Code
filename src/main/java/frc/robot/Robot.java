package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  public static DifferentialDrive drive;
  public static WPI_TalonFX br = new WPI_TalonFX(0);
  public static WPI_TalonFX fr = new WPI_TalonFX(1);
  public static WPI_TalonFX bl = new WPI_TalonFX(2);
  public static WPI_TalonFX fl = new WPI_TalonFX(3);

  public static CANSparkMax climb = new CANSparkMax(8, MotorType.kBrushed);
  public static VictorSPX hood = new VictorSPX(5);
  public static VictorSPX roller = new VictorSPX(6);
  public static VictorSPX intake = new VictorSPX(7);
  public static Spark conveyor1 = new Spark(0);
  public static Spark conveyor2 = new Spark(1);
  public static CANSparkMax flywheel = new CANSparkMax(4, MotorType.kBrushless);

  public static Compressor air = new Compressor(9);
  public static Solenoid shift = new Solenoid(9, 0);
  public static DoubleSolenoid stopper = new DoubleSolenoid(9, 2, 3);
  public static Solenoid climbPiston = new Solenoid(9, 4);

  public static XboxController remote = new XboxController(0);
  public static Timer autoTimer = new Timer();
  public static Timer intakerConveyor = new Timer();
  public static Orchestra song = new Orchestra();
  public static Joystick driveStick = new Joystick(0);
  public static int dPad;

  public static DigitalInput frontPhotoGate = new DigitalInput(0);
  public static DigitalInput upperPhotoGate = new DigitalInput(1);
  public static Encoder hoodEncoder = new Encoder(2, 3);
  public static DigitalInput upperIntakeLimit = new DigitalInput(4);
  public static DigitalInput lowerIntakeLimit = new DigitalInput(5);
  public static DigitalInput climbLimit = new DigitalInput(6);
  public static DigitalOutput lightRing = new DigitalOutput(9);

  public boolean stopperEngaged;
  public boolean flywheelSpin;
  public double flywheelSpeed;
  public boolean rollerON;
  public boolean climberPiston;
  public boolean Neo550SpinCity;
  public boolean conveyorStarted;
  public boolean shooting;
  public boolean sawIt;
  public int ballsCounter;
  public double flywheelGetVel;
  public boolean conveyorRequested;
  public boolean intakeWantConveyor;
  public double minVel;
  public boolean driveOn;
  public double flywheelMinSpeed = 0.35;

  /*
  Left Joystick = Drive
  Bumpers = Shifting
  Triggers = Intake Pivot
  Y = Intake Roller Off
  X = Intake Roller On
  A = Shoot 1 Ball
  B = Flywheel Off
  dPad Up = Climb Up
  dPad Right = Far Shot
  dPad Left = Close Shot
  Start = Climb Piston Extend
  Back = Climb Piston Retract
  */


  @Override
  public void robotInit() {
    br.setNeutralMode(NeutralMode.Coast);
    fr.setNeutralMode(NeutralMode.Coast);
    fl.setNeutralMode(NeutralMode.Coast);
    bl.setNeutralMode(NeutralMode.Coast);
    fr.configOpenloopRamp(0.5);
    fl.configOpenloopRamp(0.5);
    climberPiston = false;
    flywheelSpin = false;
    flywheelSpeed = 0.5;
    flywheel.setInverted(true);
    air.start();
    br.follow(fr);
    bl.follow(fl);
    drive = new DifferentialDrive(fl, fr);
    flywheel.getEncoder().setPosition(0);
    Neo550SpinCity = false;
    intake.setNeutralMode(NeutralMode.Brake);
    climb.setIdleMode(IdleMode.kBrake);
    flywheel.setIdleMode(IdleMode.kCoast);
    stopper.set(Value.kReverse);
  }

  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    //fr.set(TalonFXControlMode.Position, 200000.0);
    //fl.set(TalonFXControlMode.Position, 200000.0);
    autoTimer.start();
 }

  @Override
  public void autonomousPeriodic() {
    if (autoTimer.get() <= 1.5) {
      drive.arcadeDrive(-0.75, 0);
    }
    if (autoTimer.get() > 1.5) {
      drive.arcadeDrive(0, 0);
    }
  }

  public void manualDrive(double move, double turn) {
  }

  @Override
  public void teleopInit() {
    ballsCounter = 3;
    flywheel.getEncoder().setPosition(0);
    Neo550SpinCity = false;
    conveyorStarted = false;
    shooting = false;
    rollerON = false;
  }

  @Override
  public void teleopPeriodic() {

    if (flywheel.get() == 0 && shooting == true) {
      ConveyorStop();
      shooting = false;
    }

    /*
    if (dPad == 0) {
      if (driveOn == true) {
        driveOn = false;
      } else {
        driveOn = true;
      }
    }
    */

    remote.setRumble(RumbleType.kLeftRumble, flywheelGetVel / 5000);
    remote.setRumble(RumbleType.kRightRumble, flywheelGetVel / 5000);

    conveyorRequested = false;
    dPad = remote.getPOV(0);
    flywheelGetVel = flywheel.getEncoder().getVelocity();

      drive.arcadeDrive(remote.getY(Hand.kLeft), -remote.getX(Hand.kLeft));

    if (remote.getStartButtonPressed()) {
      climbPiston.set(true);
      climberPiston = true;
    }
    if (remote.getBackButtonPressed()) {
      climbPiston.set(false);
      climberPiston = false;
    }
    if (flywheelSpeed > 1) {
      flywheelSpeed = 1;
    }
    if (flywheelSpeed < 0) {
      flywheelSpeed = 0;
    }
    /*
    if (remote.getXButtonPressed()) {
      if (flywheelSpin == true) {
        flywheelSpin = false;
      } else {
        flywheelSpin = true;
      }
    }
    if (flywheelSpin == true) {
      flywheel.set(flywheelSpeed);
    }
    if (flywheelSpin == false) {
      flywheel.set(0);
    }
        if (remote.getAButtonPressed()) {
      if (rollerON == true) {
        rollerON = false;
      } else {
        rollerON = true;
      }
    }
    if (rollerON == true) {
      roller.set(ControlMode.PercentOutput, 0.5);
    }
    if (rollerON == false) {
      roller.set(ControlMode.PercentOutput, 0);
    }
    */
    if (remote.getBumperPressed(Hand.kRight)) {
      shift.set(true);
    }
    if (remote.getBumperPressed(Hand.kLeft)) {
      shift.set(false);
    }
    if (remote.getTriggerAxis(Hand.kRight) > 0.2) {
      intake.set(ControlMode.PercentOutput, -0.3);
    }
    if (remote.getTriggerAxis(Hand.kLeft) > 0.2) {
      intake.set(ControlMode.PercentOutput, 0.5);
    }
    if (remote.getTriggerAxis(Hand.kRight) < 0.1 && remote.getTriggerAxis(Hand.kLeft) < 0.1) {
      intake.set(ControlMode.PercentOutput, 0);
    }
    /*
     * if (remote.getXButtonPressed()) { Neo550SpinCity = true; } if
     * (flywheel.getEncoder().getPosition() <= 85 && Neo550SpinCity == true) {
     * flywheel.set(flywheelSpeed); } if (flywheel.getEncoder().getPosition() > 85)
     * { flywheel.set(0); }
     */
    if (remote.getBButtonPressed()) {flywheel.set(0);}
    if (dPad == 270) {
      hood.set(ControlMode.PercentOutput, -1);
      flywheel.set(flywheelMinSpeed);
      // should be 0.45
      minVel = 1000;
    }
    else hood.set(ControlMode.PercentOutput, 0);
    if (dPad == 90) {
      hood.set(ControlMode.PercentOutput, 1);
      flywheel.set(1);
      minVel = 4500;
    }
    else hood.set(ControlMode.PercentOutput, 0);
    flywheelSpin = flywheel.get() != 0;

    if (dPad == 180 && climbLimit.get() == true) {climb.set(-0.5);}
    else {
      climb.set(0);
    }

    if (frontPhotoGate.get() == true) {
      intakeWantConveyor = true;
      intakerConveyor.reset();
      intakerConveyor.start();
      BallCounterUp();
    }
    if (intakerConveyor.get() >= 0.2) {intakeWantConveyor = false;}
    if (intakeWantConveyor) {conveyorRequested = true;}

    if (remote.getAButtonPressed() && flywheelGetVel > minVel && shooting == false) {
      intakerConveyor.stop();
      intakerConveyor.reset();
      shooting = true;
      BallCounterDown();
    }
    if (shooting == true) {
      if (upperPhotoGate.get() == true) {
        ConveyorStart();
        sawIt = true;
      }
      if (sawIt == true && upperPhotoGate.get() == false) {
        ConveyorStop();
        shooting = false;
        sawIt = false;
      }
    }
    if (shooting) {conveyorRequested = true;}
    hood.set(ControlMode.PercentOutput, -remote.getY(Hand.kRight));

    if(conveyorRequested) ConveyorStart();
    else ConveyorStop();

    //stopper.set(shooting ? Value.kReverse : Value.kForward);
    if (remote.getYButtonPressed()) {
      rollerON = false;
    }
    if (remote.getXButtonPressed()) {
      rollerON = true;
    }
    if (rollerON == true) {
      roller.set(ControlMode.PercentOutput, -0.4);
    }
    if (rollerON == false) {
      roller.set(ControlMode.PercentOutput, 0);
    }
    SmartDashboard.putNumber("RPM", flywheelGetVel);
    SmartDashboard.putBoolean("FlyWheel Running", flywheelSpin);
    SmartDashboard.putNumber("Flywheel Speed", flywheelSpeed);
    SmartDashboard.putBoolean("Stopper Engaged", stopper.get() == Value.kReverse);
    SmartDashboard.putBoolean("Conveyor Running", conveyor1.get() != 0);
    SmartDashboard.putBoolean("RollerRunning", rollerON);
    SmartDashboard.putNumber("Climb Motor Rotations", flywheelGetVel);
    SmartDashboard.putBoolean("Climb Piston", climberPiston);
    SmartDashboard.putBoolean("LowPhotoGate", frontPhotoGate.get());
    SmartDashboard.putBoolean("HighPhotoGate", upperPhotoGate.get());
    SmartDashboard.putNumber("Balls In", ballsCounter);
    SmartDashboard.putNumber("Timer", intakerConveyor.get());
    SmartDashboard.putNumber("hood Encoder", hoodEncoder.getDistance());
    SmartDashboard.putNumber("NEO Temp", flywheel.getMotorTemperature());
    SmartDashboard.putNumber("Neo Current", flywheel.getOutputCurrent());
    SmartDashboard.putBoolean("Climb Limit", climbLimit.get());
    SmartDashboard.putBoolean("Upper intake limit", upperIntakeLimit.get());
    SmartDashboard.putBoolean("High Gear", shift.get());
    SmartDashboard.putNumber("Lift Current", climb.getOutputCurrent());
    SmartDashboard.putBoolean("Drive On?", driveOn);
  }

  public void ConveyorStart() {
    conveyor1.set(-1);
    conveyor2.set(1);
  }

  public void ConveyorStop() {
    conveyor1.set(0);
    conveyor2.set(0);
  }

  public void BallCounterUp() {
    if (ballsCounter < 5) {
      ballsCounter++;
    }
  }

  public void BallCounterDown() {
    if (ballsCounter > 0) {
      ballsCounter--;
    }
  }

  @Override
  public void testPeriodic() {
  }
}
