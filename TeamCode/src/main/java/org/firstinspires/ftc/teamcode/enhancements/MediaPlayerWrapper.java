package org.firstinspires.ftc.teamcode.driverchoices;

import android.media.MediaPlayer;

import org.firstinspires.ftc.teamcode.debugging.ConsoleManager;

public class MediaPlayerWrapper
{
    private static MediaPlayer mediaPlayer = null;
    protected static enum DownloadedSongs
    {
        IMPERIAL_MARCH(com.qualcomm.ftcrobotcontroller.R.raw.imperialmarch);

        private final int audio;
        DownloadedSongs(int audio)
        {
            this.audio = audio;
        }

        public int getAudio()
        {
            return this.audio;
        }
    }

    protected static void playAudio(DownloadedSongs choice)
    {
        if(isPlaying()) // TODO: Necessary? It might just automatically  stop the audio.
        {
            stopAudio();
        }

        try
        {
            mediaPlayer = MediaPlayer.create(hardwareMap.appContext, choice.getAudio());
            mediaPlayer.start();
            mediaPlayer.setOnCompletionListener(new MediaPlayer.OnCompletionListener()
            {
                public void onCompletion(MediaPlayer player)
                {
                    player.release();
                }
            });

            ConsoleManager.outputNewLineToDrivers("Playing " + choice.toString());
            ProgramFlow.pauseForMS(1000); /* Let MediaPlayer initialize; register that song is playing. */
        }
        catch (Exception e)
        {
            ConsoleManager.outputNewLineToDrivers("Error while attempting to play music.");
            return;
        }
    }

    /**
     * Stop audio and free memory.
     */
    protected static void stopAudio()
    {
        if (mediaPlayer != null)
        {
            if (mediaPlayer.isPlaying())
            {
                mediaPlayer.stop();
            }
            mediaPlayer.release();
            mediaPlayer = null;
        }
    }

    public static boolean isPlaying()
    {
        return mediaPlayer != null;
    }
}
