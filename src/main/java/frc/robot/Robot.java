package frc.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	final WPI_TalonFX br = new WPI_TalonFX(0);
	final WPI_TalonFX fr = new WPI_TalonFX(1);
	final WPI_TalonFX bl = new WPI_TalonFX(2);
	final WPI_TalonFX fl = new WPI_TalonFX(3);
	final DifferentialDrive drive = new DifferentialDrive(fl, fr);

	final CANSparkMax climb = new CANSparkMax(8, MotorType.kBrushed);
	final WPI_VictorSPX hood = new WPI_VictorSPX(5);
	final WPI_VictorSPX roller = new WPI_VictorSPX(6);
	final WPI_VictorSPX intake = new WPI_VictorSPX(7);
	final Spark conveyor1 = new Spark(0);
	final Spark conveyor2 = new Spark(1);
	final CANSparkMax flywheel = new CANSparkMax(4, MotorType.kBrushless);

	final Compressor air = new Compressor(9);
	final Solenoid shift = new Solenoid(9, 0);
	final DoubleSolenoid stopper = new DoubleSolenoid(9, 2, 3);
	final Solenoid climbPiston = new Solenoid(9, 4);

	final XboxController remote = new XboxController(0);
	final Joystick driveStick = new Joystick(0);

	final Gyro gyro = new ADIS16448_IMU();

	final DigitalInput frontPhotoGate = new DigitalInput(0);
	final DigitalInput upperPhotoGate = new DigitalInput(1);
	final Encoder hoodEncoder = new Encoder(2, 3);
	final DigitalInput upperIntakeLimit = new DigitalInput(4);
	final DigitalInput lowerIntakeLimit = new DigitalInput(5);
	final DigitalInput climbLimit = new DigitalInput(6);
	final DigitalOutput lightRing = new DigitalOutput(9);

	NetworkTableEntry poseEntry;
	final PIDController autoAlignPID = new PIDController(0.01, 0, 0);

	final Timer autoTimer = new Timer();
	final Timer intakerConveyor = new Timer();

	boolean flywheelSpin;
	double flywheelSpeed = 0.5;
	boolean rollerON;
	boolean climberPiston;
	boolean Neo550SpinCity;
	boolean conveyorStarted;
	boolean shooting;
	boolean sawIt;
	int ballsCounter = 3;
	boolean intakeWantConveyor;
	double minVel;
	boolean driveOn;
	boolean autoAlignEnabled;
	static final double flywheelMinSpeed = 0.35;

	/*
	 * Left Joystick = Drive
	 * Bumpers = Shifting
	 * Triggers = Intake Pivot
	 * A = Shoot 1 Ball
	 * B = Flywheel Off
	 * X = Intake Roller On
	 * Y = Intake Roller Off
	 * dPad Up = Climb Up
	 * dPad Left = Close Shot
	 * dPad Right = Far Shot
	 * Start = Climb Piston Extend
	 * Back = Climb Piston Retract
	 */

	@Override
	public void robotInit() {
		br.setNeutralMode(NeutralMode.Coast);
		fr.setNeutralMode(NeutralMode.Coast);
		fl.setNeutralMode(NeutralMode.Coast);
		bl.setNeutralMode(NeutralMode.Coast);
		br.follow(fr);
		bl.follow(fl);
		fr.configOpenloopRamp(0.5);
		fl.configOpenloopRamp(0.5);

		flywheel.setInverted(true);
		flywheel.getEncoder().setPosition(0);
		flywheel.setIdleMode(IdleMode.kCoast);

		air.start();
		intake.setNeutralMode(NeutralMode.Brake);
		climb.setIdleMode(IdleMode.kBrake);
		stopper.set(Value.kReverse);

		final NetworkTableInstance inst = NetworkTableInstance.getDefault();
		final NetworkTable table = inst.getTable("chameleon-vision/Microsoft_LifeCam_HD_3000");
		poseEntry = table.getEntry("targetPose");

		autoAlignPID.setSetpoint(0);
	}

	public void robotPeriodic() {
	}

	@Override
	public void autonomousInit() {
		autoTimer.start();
	}

	@Override
	public void autonomousPeriodic() {
		drive.arcadeDrive(autoTimer.get() <= 1.5 ? 0.75 : 0, 0);
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {

		final int dPad = remote.getPOV(0);
		final double flywheelGetVel = flywheel.getEncoder().getVelocity();
		boolean conveyorRequested = false;

		final Translation2d targetTranslation = getTargetPose().getTranslation();
		final double targetAngle = Math.atan2(targetTranslation.getY(), targetTranslation.getX());
		if (dPad == 0) autoAlignEnabled = !autoAlignEnabled;
		if (autoAlignEnabled) {
			fl.setVoltage(autoAlignPID.calculate(targetAngle));
			fr.setVoltage(-autoAlignPID.calculate(targetAngle));
		}

		if (flywheel.get() == 0 && shooting) {
			ConveyorStop();
			shooting = false;
		}

		// if (dPad == 0) driveOn = !driveOn;

		remote.setRumble(RumbleType.kLeftRumble, flywheelGetVel / 5000);
		remote.setRumble(RumbleType.kRightRumble, flywheelGetVel / 5000);

		remote.setRumble(RumbleType.kLeftRumble, flywheelGetVel / 4700);
		remote.setRumble(RumbleType.kRightRumble, flywheelGetVel / 4700);
		if (driveOn == true) {
			drive.arcadeDrive(remote.getY(Hand.kLeft), -remote.getX(Hand.kLeft));
		}

		if (remote.getStartButtonPressed()) {
			climbPiston.set(true);
			climberPiston = true;
		}
		if (remote.getBackButtonPressed()) {
			climbPiston.set(false);
			climberPiston = false;
		}
		if (flywheelSpeed > 1) flywheelSpeed = 1;
		if (flywheelSpeed < 0) flywheelSpeed = 0;

		// if (remote.getXButtonPressed()) flywheelSpin = !flywheelSpin;
		// flywheel.set(flywheelSpin ? flywheelSpeed : 0);
		// if (remote.getAButtonPressed()) rollerON = !rollerON;
		// roller.set(rollerON ? 0.5 : 0);

		if (remote.getBumperPressed(Hand.kRight)) shift.set(true);
		if (remote.getBumperPressed(Hand.kLeft)) shift.set(false);
		if (remote.getTriggerAxis(Hand.kRight) > 0.2) intake.set(-0.3);
		if (remote.getTriggerAxis(Hand.kLeft) > 0.2) intake.set(0.5);
		if (remote.getTriggerAxis(Hand.kRight) < 0.1 && remote.getTriggerAxis(Hand.kLeft) < 0.1) intake.set(0);

		// if (remote.getXButtonPressed()) Neo550SpinCity = true;
		// flywheel.set(flywheel.getEncoder().getPosition() <= 85 && Neo550SpinCity ? flywheelSpeed : 0);

		if (remote.getBButtonPressed()) flywheel.set(0);
		if (dPad == 270) {
			hood.set(-1);
			flywheel.set(flywheelMinSpeed);
			// should be 0.45
			minVel = 1000;
		}
		else if (dPad == 90) {
			hood.set(1);
			flywheel.set(1);
			minVel = 4500;
		}
		else hood.set(0);

		flywheelSpin = flywheel.get() != 0;

		climb.set(dPad == 180 && climbLimit.get() ? -0.5 : 0);

		if (frontPhotoGate.get() == true) {
			intakeWantConveyor = true;
			intakerConveyor.reset();
			intakerConveyor.start();
			BallCounterUp();
		}
		if (intakerConveyor.get() >= 0.2) intakeWantConveyor = false;

		if (remote.getAButtonPressed() && flywheelGetVel > minVel && shooting == false) {
			intakerConveyor.stop();
			intakerConveyor.reset();
			shooting = true;
			BallCounterDown();
		}
		if (shooting) {
			if (upperPhotoGate.get()) {
				ConveyorStart();
				sawIt = true;
			}
			else if (sawIt) {
				ConveyorStop();
				shooting = false;
				sawIt = false;
			}
		}

		conveyorRequested = intakeWantConveyor || shooting;

		hood.set(-remote.getY(Hand.kRight));

		if (conveyorRequested) ConveyorStart();
		else ConveyorStop();

		// stopper.set(shooting ? Value.kReverse : Value.kForward);
		if (remote.getYButtonPressed()) rollerON = false;
		if (remote.getXButtonPressed()) rollerON = true;
		roller.set(rollerON ? -0.4 : 0);

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
		SmartDashboard.putNumberArray("Target Pose", poseEntry.getDoubleArray(new double[3]));
	}

	@Override
	public void testPeriodic() {
	}

	private void ConveyorStart() {
		conveyor1.set(-1);
		conveyor2.set(1);
	}

	private void ConveyorStop() {
		conveyor1.set(0);
		conveyor2.set(0);
	}

	private void BallCounterUp() {
		if (ballsCounter < 5) ++ballsCounter;
	}

	private void BallCounterDown() {
		if (ballsCounter > 0) --ballsCounter;
	}

	private Pose2d getTargetPose() {
		double[] poseArray = poseEntry.getDoubleArray(new double[3]);
		return new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[2]));
	}
}
