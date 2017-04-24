package org.makiah.niftc.examples;

import org.makiah.niftc.NiFTBase;
import org.makiah.niftc.music.NiFTMusic;
import org.makiah.niftc.threads.NiFTFlow;

public class StarWars extends NiFTBase
{
    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        //Play Imperial March, could also be a debugging tone.
        NiFTMusic.play (NiFTMusic.AudioFiles.IMPERIAL_MARCH);

        //Typically this is where the robot would do things, but we'll just have it wait until the user stops it.
        while (true)
            NiFTFlow.pauseForSingleFrame ();
    }
}
