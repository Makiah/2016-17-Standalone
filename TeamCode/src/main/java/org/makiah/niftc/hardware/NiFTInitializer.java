package org.makiah.niftc.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.makiah.niftc.NiFTBase;

/**
 * Initializes any hardware device provided by some class, if it exists o_O
 */
public class NiFTInitializer
{
    private static HardwareMap newHardwareMap;
    public static void initializeWith(HardwareMap hardwareMap)
    {
        newHardwareMap = hardwareMap;
    }

    public static <T extends HardwareDevice> T initialize (Class<T> hardwareDevice, String name)
    {
        try
        {
            //Returns the last subclass (if this were a DcMotor it would pass back a Dc Motor.
            return hardwareDevice.cast (newHardwareMap.get (name));
        }
        catch (Exception e) //There might be other exceptions that this throws, not entirely sure about which so I am general here.
        {
            throw new NullPointerException ("Couldn't find " + name + " in the configuration file!");
        }
    }
}
