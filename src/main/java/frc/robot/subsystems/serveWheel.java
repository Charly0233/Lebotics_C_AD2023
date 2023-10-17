package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;


public class serveWheel {
    //motors
    private static int numberWheel;
    private static CANSparkMax turnMotor;
    private static CANSparkMax speedMotor;
   //encoders
    private static CANCoder turnEncoder;

    private PIDController pid;
    double maxAngle = 180;

    public serveWheel(int speedId, int angleId, int angleEncoder,int numberWheel){
        serveWheel.numberWheel=numberWheel;
        turnMotor = new CANSparkMax(angleId, MotorType.kBrushless);
        speedMotor = new CANSparkMax(speedId, MotorType.kBrushless);
        turnEncoder= new CANCoder(angleEncoder);

        turnEncoder.setPositionToAbsolute();
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        pid = new PIDController(.00125,.005, 0);
    }
    /**
 * Get the closest angle between the given angles.
 */
private static double closestAngle(double a, double b)
{
        // get direction
        double dir = modulo(b, 360.0) - modulo(a, 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
}


    public void setDirection(double setpoint, double motorSpeed)
    {
        double currentAngle = turnEncoder.getAbsolutePosition();
        // find closest angle to setpoint
        double setpointAngle = closestAngle(currentAngle, setpoint);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            speedMotor.set(motorSpeed);
            turnMotor.set(pid.calculate(currentAngle, setpointAngle));
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            speedMotor.set(-motorSpeed);
            turnMotor.set(pid.calculate(currentAngle, setpointAngleFlipped));
        }
    }

    public static void update(){
        SmartDashboard.putNumber("Wheel "+numberWheel+" encoder at:", turnEncoder.getAbsolutePosition());
    }
}
