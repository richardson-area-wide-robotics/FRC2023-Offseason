package frc.lib.encoders;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class AnalogAbsoluteEncoder extends SwerveEncoder{
 
  /**
   * Encoder as Analog Input.
   */
  public AnalogInput encoder;
  /**
   * Inversion state of the encoder.
   */
  private boolean inverted = false;

  /**
   * Construct the Thrifty Encoder as a Swerve Absolute Encoder.
   *
   * @param encoder Encoder to construct.
   */
  public AnalogAbsoluteEncoder(AnalogInput encoder)
  {
    this.encoder = encoder;
  }

  /**
   * Construct the Encoder given the analog input channel.
   *
   * @param channel Analog Input channel of which the encoder resides.
   */
  public AnalogAbsoluteEncoder(int channel)
  {
    this(new AnalogInput(channel));
  }

  /**
   * Reset the encoder to factory defaults.
   */
  @Override
  public void factoryDefault()
  {
    // Do nothing
  }

  /**
   * Clear sticky faults on the encoder.
   */
  @Override
  public void clearStickyFaults()
  {
    // Do nothing
  }

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {
    this.inverted = inverted;
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    return (inverted ? -1.0 : 1.0) * (encoder.getAverageVoltage() / RobotController.getVoltage5V()) * 360;
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  @Override
  public Object getAbsoluteEncoder()
  {
    return encoder;
  }
    
}
