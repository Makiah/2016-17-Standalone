package org.firstinspires.ftc.teamcode.niftc.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.niftc.NiFTBase;

/**
 * Initializes any hardware device provided by some class, if it exists o_O
 */
public class NiFTInitializer
{
    /**
     * Use by calling DcMotor dcMotor = NiFTInitializer.initialize(DcMotor.class, "name in config");
     */
    public static <T extends HardwareDevice> T initialize (Class<T> hardwareDevice, String name)
    {
        try
        {
            //Returns the last subclass (if this were a DcMotor it would pass back a Dc Motor.
            return hardwareDevice.cast (NiFTBase.opModeInstance.hardwareMap.get (name));
        }
        catch (Exception e) //There might be other exceptions that this throws, not entirely sure about which so I am general here.
        {
            throw new NullPointerException ("Couldn't find " + name + " in the configuration file!");
        }
    }
}
