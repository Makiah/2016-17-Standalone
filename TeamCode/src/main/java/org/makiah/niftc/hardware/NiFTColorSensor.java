package org.makiah.niftc.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.makiah.niftc.threads.NiFTFlow;

/**
 * This class currently only encapsulates the color sensor, and enables immediate I2C address setting and LED enabling.
 */
public class NiFTColorSensor
{
    public final ColorSensor sensor;

    /**
     * Initializes the color sensor with given traits.
     *
     * @param colorSensorName The name of the sensor in the config.
     * @param i2cAddress The i2c address of the sensor.
     * @param enableLED Whether or not the led should be enabled.
     * @throws InterruptedException The program might terminate while this is running.
     */
    public NiFTColorSensor (String colorSensorName, int i2cAddress, boolean enableLED) throws InterruptedException
    {
        this.sensor = NiFTInitializer.initialize (ColorSensor.class, colorSensorName);

        sensor.setI2cAddress (I2cAddr.create8bit (i2cAddress));

        sensor.enableLed (false);
        if (enableLED)
        {
            NiFTFlow.pauseForMS (200);
            sensor.enableLed (true);
        }
    }
}
