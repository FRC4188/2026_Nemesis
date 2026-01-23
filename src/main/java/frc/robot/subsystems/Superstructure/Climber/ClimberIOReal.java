// package frc.robot.subsystems.Superstructure.Climber;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import static edu.wpi.first.units.Units.Hertz;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Temperature;
// import edu.wpi.first.units.measure.Voltage;
// import frc.robot.Constants;

// public class ClimberIOReal implements ClimberIO{
//     private final TalonFX motor;

//     private final StatusSignal<Voltage> appliedVolts;
//     private final StatusSignal<Temperature> tempC;
//     private final StatusSignal<Angle> posRots;

//     private final VoltageOut voltReq = new VoltageOut(0);
//     private final VoltageOut voltFOC = new VoltageOut(0).withEnableFOC(true);

//     public ClimberIOReal(){
//         motor = new TalonFX(Constants.Id.kClimber, Constants.Robot.rio);

//         motor.clearStickyFaults();
//         motor.getConfigurator().apply(Constants.IntakeConstants.kMotorConfig);//change this, it
// is ugly
//         motor.optimizeBusUtilization();

//         posRots = motor.getPosition();
//         appliedVolts = motor.getMotorVoltage();
//         tempC = motor.getDeviceTemp();
//         posRots.setUpdateFrequency(Hertz.of(50));
//         appliedVolts.setUpdateFrequency(Hertz.of(50));
//         tempC.setUpdateFrequency(Hertz.of(0.5))
//     }

// }
