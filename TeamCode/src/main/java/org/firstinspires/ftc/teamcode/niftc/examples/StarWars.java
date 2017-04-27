package org.firstinspires.ftc.teamcode.niftc.examples;

import org.firstinspires.ftc.teamcode.niftc.NiFTBase;
import org.firstinspires.ftc.teamcode.niftc.music.NiFTMusic;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;

public class StarWars extends NiFTBase
{
    /**
     * Another example of a really easy class which literally just plays the imperial march indefinitely :P
     */
    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        //Play Imperial March, could also be a debugging tone.
        NiFTMusic.play (NiFTMusic.AudioFiles.IMPERIAL_MARCH);

        //Typically this is where the robot would do things, but we'll just have it wait until the user stops it.
        while (true)
            NiFTFlow.yieldForFrame ();
    }
}
