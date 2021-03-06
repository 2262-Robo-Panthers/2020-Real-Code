package frc.robot;

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

	final DigitalInput frontPhotoGate = new DigitalInput(0);
	final DigitalInput upperPhotoGate = new DigitalInput(1);
	final DigitalInput otherPhotoGate = new DigitalInput(7);
	final Encoder hoodEncoder = new Encoder(2, 3);
	final DigitalInput upperIntakeLimit = new DigitalInput(4);
	final DigitalInput lowerIntakeLimit = new DigitalInput(5);
	final DigitalInput climbLimit = new DigitalInput(6);
	final DigitalOutput lightRing = new DigitalOutput(9);

	NetworkTableEntry targetInViewEntry;
	NetworkTableEntry poseEntry;
	final PIDController autoAlignPID = new PIDController(1, 0, 0);

	final Timer autoTimer = new Timer();
	final Timer initiationLineTimer = new Timer();
	final Timer flywheelStarting = new Timer();

	final PIDController flywheelPID = new PIDController(1, 0, 0);

	boolean flywheelSpin;
	double flywheelSpeed = 0.5;
	int flywheelSetpoint = 0;
	boolean rollerON;
	boolean climberPiston;
	boolean Neo550SpinCity;
	boolean conveyorStarted;
	boolean shooting;
	boolean sawIt;
	boolean intakeWantConveyor = false;
	double minVel;
	boolean autoAlignEnabled;
	boolean targetInView;
	boolean inPosition;
	boolean intaking;
	boolean intakingParty;
	boolean flywheelWantToShoot;
	boolean flywheelCanShoot;
	boolean dPadPress;
	boolean runDpadMethod;
	double flywheelMinSpeed = 0;
    boolean dPadWasUp = false;
    boolean rightStickWasPressed = false;

	/*
	 * Left Joystick = Drive
	 * Bumpers = Shifting
	 * Triggers = Intake Pivot
	 * A = Shoot 1 Ball
	 * B = Flywheel Off
	 * X = Intake Roller On
	 * Y = Intake Roller Off
	 * D-Pad Up = Auto Align
	 * D-Pad Down = Climb Up
	 * D-Pad Left = Close Shot
	 * D-Pad Right = Far Shot
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

		flywheelPID.setSetpoint(1700);

		air.start();
		intake.setNeutralMode(NeutralMode.Brake);
		climb.setIdleMode(IdleMode.kBrake);

		final NetworkTableInstance inst = NetworkTableInstance.getDefault();
		final NetworkTable table = inst.getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
		targetInViewEntry = table.getEntry("isValid");
		poseEntry = table.getEntry("targetPose");

		autoAlignPID.setSetpoint(0);
	}

	public void robotPeriodic() {

		// switch (flywheelSetpoint) {
		// 	case 0:
		// 		flywheel.setVoltage(0);
		// 		break;
		// 	case 1:
		// 		flywheel.setVoltage(flywheelPID.calculate(flywheel.getEncoder().getVelocity()) + 4);
		// 		break;
		// 	case 2:
		// 		flywheel.setVoltage(12);
		// 		break;
		// 	default:
		// 		flywheel.setVoltage(0);
		// 		break;
		// }
		
		flywheelSpin = flywheel.get() != 0;
		SmartDashboard.putNumber("RPM", flywheel.getEncoder().getVelocity());
		SmartDashboard.putBoolean("Flywheel Running", flywheelSpin);
		SmartDashboard.putNumber("NEO Temp", flywheel.getMotorTemperature());
		SmartDashboard.putNumber("Neo Current", flywheel.getOutputCurrent());
		SmartDashboard.putBoolean("Conveyor Running", conveyor1.get() != 0);
		SmartDashboard.putBoolean("Fast Gear", shift.get());
	}

	@Override
	public void autonomousInit() {
		targetInView = false;
		initiationLineTimer.start();
		initiationLineTimer.reset();
        	// flywheelSetpoint = 1;
        	flywheel.set(0.4);
		stopper.set(Value.kForward);
	}

	@Override
	public void autonomousPeriodic() {

        flywheel.set(0.4);

		if (initiationLineTimer.get() < 1.5) drive.arcadeDrive(0.5, 0);
		
		else {
			if (getTargetInView()) {
				drive.arcadeDrive(-0.5, 0);
			}
			else if (targetInView) {
				inPosition = true;
				autoTimer.start();
				autoTimer.reset();
			}

			if (inPosition = true) {
				drive.arcadeDrive(0, 0);
			}

			if (inPosition && autoTimer.get() < 10.0 && flywheel.getEncoder().getVelocity() > 1800) ConveyorGo();
			else ConveyorStop();
		}

		// if (autoTimer.get() < 3.0) flywheelSetpoint = 0;
		//flywheel.set(autoTimer.get() < 3.0 ? 0.4 : 0);

		SmartDashboard.putBoolean("Target In View", targetInView);
		targetInView = getTargetInView();
	}

	@Override
	public void teleopInit() {
        ConveyorStop();
        flywheel.set(0);
		flywheel.getEncoder().setPosition(0);
		flywheelSetpoint = 0;
		flywheelSpin = false;
		Neo550SpinCity = false;
		conveyorStarted = false;
		shooting = false;
		rollerON = false;
		autoAlignEnabled = false;
		intakeWantConveyor = false;
		intakingParty = false;
		flywheelMinSpeed = 0;
	}

	@Override
	public void teleopPeriodic() {

		if (shooting == false) {
			stopper.set(Value.kReverse);
		}
		else {
			stopper.set(Value.kForward);
		}

		drive.arcadeDrive(remote.getY(Hand.kLeft), -remote.getX(Hand.kLeft)/2);

		final int dPad = remote.getPOV(0);
		final double flywheelGetVel = flywheel.getEncoder().getVelocity();

		final Translation2d targetTranslation = getTargetPose().getTranslation();
		final double targetAngle = -Math.atan2(targetTranslation.getY(), targetTranslation.getX());
		/*
		if (dpad == 180) autoAlignEnabled = !autoAlignEnabled;
		if (autoAlignEnabled) {
			fl.setVoltage(autoAlignPID.calculate(targetAngle));
			fr.setVoltage(-autoAlignPID.calculate(targetAngle));
		}
		*/

		if (flywheel.getEncoder().getVelocity() < 1300 || flywheel.get() == 0) shooting = false;

		remote.setRumble(RumbleType.kLeftRumble, flywheelGetVel / 4700);
		remote.setRumble(RumbleType.kRightRumble, flywheelGetVel / 4700);

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

		if (remote.getBumperPressed(Hand.kRight)) {
			shift.set(true);
			//stopper.set(Value.kForward);
		}
		if (remote.getBumperPressed(Hand.kLeft)){
			shift.set(false);
			//stopper.set(Value.kReverse);
		}
		if (remote.getTriggerAxis(Hand.kRight) > 0.2) intake.set(-0.3);
		if (remote.getTriggerAxis(Hand.kLeft) > 0.2) intake.set(0.5);
		if (remote.getTriggerAxis(Hand.kRight) < 0.1 && remote.getTriggerAxis(Hand.kLeft) < 0.1) intake.set(0);

		// if (remote.getXButtonPressed()) Neo550SpinCity = true;
		// flywheel.set(flywheel.getEncoder().getPosition() <= 85 && Neo550SpinCity ? flywheelSpeed : 0);

		if (remote.getBButtonPressed()){
			//flywheelWantToShoot = false;
			flywheelMinSpeed = 0;
			ConveyorStop();
		}
		if (dPad == 270) {
			dPadPress = true;
			flywheelMinSpeed = 0.45;
			//flywheelWantToShoot = true;
			// flywheelSetpoint = 1;
            // should be 0.45
			minVel = 1600;
		}
        if (dPad == 90) {
			dPadPress = true;
			//flywheelWantToShoot = true;
			flywheelMinSpeed = 1;
			// flywheelSetpoint = 2;
			minVel = 4500;
		}
		if (dPad == 180) {
			ConveyorReverse();
		}
        else if (dPadWasUp) ConveyorStop();
        
        if (remote.getStickButton(Hand.kRight)) {
            ConveyorSlow();
        }
        else if (rightStickWasPressed) ConveyorStop();

		/*
		if (shooting == false && upperPhotoGate.get() == true) {
			ConveyorStop();
		}
		*/

		flywheel.set(flywheelMinSpeed);

		/*
		if (flywheelWantToShoot == true) {
			FlywheelAllowed();
		}
		}
		else flywheel.set(0);
		*/

		flywheelSpin = flywheel.get() != 0;

		climb.set(dPad == 0 && climbLimit.get() ? -0.5 : 0);
		
		// if (otherPhotoGate.get() == true) intakeWantConveyor = false;
		// if (frontPhotoGate.get() == true) {
		// 	intakeWantConveyor = true;
		// 	intakerConveyor.reset();
		// 	intakerConveyor.start();
		// 	BallCounterUp();
		// }
		// //if (intakerConveyor.get() >= 1) intakeWantConveyor = false;

		// if (remote.getAButtonPressed() && flywheelGetVel > minVel && !shooting) {
		// 	intakerConveyor.stop();
		// 	intakerConveyor.reset();
		// 	shooting = true;
		// 	BallCounterDown();
		// }
		// if (shooting) {
		// 	if (upperPhotoGate.get()) {
		// 		conveyorRequested = true;
		// 		sawIt = true;
		// 	}
		// 	else if (sawIt) {
		// 		shooting = false;
		// 		sawIt = false;
		// 	}
		// }

		// conveyorRequested = intakeWantConveyor || shooting;

		// if (conveyorRequested== true) ConveyorStall();
		// // if (conveyorRequested== true && intakeWantConveyor == false) ConveyorStart();
		// else ConveyorStop();
		

		
		// if (frontPhotoGate.get() == true) {
		// 	ConveyorIntake();
		// 	intaking = true;
		// }
		// if (frontPhotoGate.get() == false && intaking == true && otherPhotoGate.get() == false) {
		// 	intaking = false;
		// 	ConveyorGo();
		// }
		// if (otherPhotoGate.get() == true && intaking == false) {
		// 	ConveyorStop();
		// }
		

		if (frontPhotoGate.get()) {
			intakingParty = true;
		}
		if (intakingParty == true) {
			Intaking();
		}

		if (flywheel.getEncoder().getVelocity() > 1800 && remote.getAButtonPressed()) {
			ConveyorGo();
			shooting = true;
		}
		if (shooting && upperPhotoGate.get()) {
			sawIt = true;
		}
		if (!upperPhotoGate.get() && shooting && sawIt) {
			ConveyorStop();
			shooting = false;
			sawIt = false;
		}
		/*
		if (shooting = false) {
			stopper.set(Value.kForward);
		}
		else {
			stopper.set(Value.kReverse);
		}
		*/
		hood.set(-remote.getY(Hand.kRight));
		if (remote.getYButtonPressed()) {
			rollerON = false;
			//flywheelMinSpeed = 0;
			}
		if (remote.getXButtonPressed()) {
			rollerON = true;
			//flywheelMinSpeed = 0.2;
		}
		roller.set(rollerON ? -0.4 : 0);

        dPadWasUp = dPad == 180;
        rightStickWasPressed = remote.getStickButton(Hand.kRight);

		SmartDashboard.putBoolean("RollerRunning", rollerON);
		SmartDashboard.putBoolean("Climb Piston", climberPiston);
		SmartDashboard.putNumber("Angle To Target", targetAngle);
	}

	@Override
	public void testPeriodic() {
	}


	private void Intaking() {
		if (frontPhotoGate.get()) {
			ConveyorIntake();
			intaking = true;
		}
		if (!frontPhotoGate.get() && intaking && !otherPhotoGate.get()) {
			intaking = false;
			ConveyorGo();
		}
		if (otherPhotoGate.get() && !intaking) {
			ConveyorStop();
			intakingParty = false;
		}
	}
	private void FlywheelAllowed() {
		if (flywheelWantToShoot == true && upperPhotoGate.get() == false) {
			flywheelCanShoot = true;
		}
		if (flywheelWantToShoot == false) {
			flywheelCanShoot = false;
		}
		if (flywheelWantToShoot == true && upperPhotoGate.get() == true) {
			ConveyorReverse();
		}

	}
	private void ConveyorGo() {
		conveyor1.set(-1);
		conveyor2.set(1);
	}
	private void dPadPressed() {
		if (dPadPress == true) {
			ConveyorReverse();
		}
		if (dPadPress == false) {
			ConveyorStop();
			runDpadMethod = false;
		}
	}

	private void ConveyorStop() {
		conveyor1.set(0);
		conveyor2.set(0);
	}
	private void ConveyorReverse() {
		conveyor1.set(1);
		conveyor2.set(-1);
	}

	private void ConveyorIntake() {
		conveyor1.set(-1);
		conveyor2.set(-0.4);
    }
    
    private void ConveyorSlow() {
        conveyor1.set(-0.5);
        conveyor2.set(0.5);
    }

	private Pose2d getTargetPose() {
		double[] poseArray = poseEntry.getDoubleArray(new double[3]);
		return new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[2]));
	}

	private boolean getTargetInView() {
		return targetInViewEntry.getBoolean(false);
	}
}
