/* Copyright (c) 2014 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.mainRobotPrograms;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.ArrayList;

//Edited in order to have all of the important constants as final, so that no unintentional modifications are made.
//This class should be used so that any changes made to the robot configuration propagates through all parts of the code that has been written.

public abstract class RobotBase extends LinearOpMode
{
    /*** CONFIGURE ALL ROBOT ELEMENTS HERE ***/
    //Drive motors
    protected DcMotor left, right;
    //Other motors
    protected DcMotor harvester, pusher;

    //This took a LONG TIME TO WRITE
    protected <T extends HardwareDevice> T Initialize  (Class <T> hardwareDevice, String name)
    {
        try
        {
            //Returns the last subclass (if this were a DcMotor it would pass back a Dc Motor.
            return hardwareDevice.cast(hardwareMap.get(name));
        }
        catch (Exception e)
        {
            OutputToDriverStation("Could not find " + name + " in hardware map.");
            return null;
        }
    }

    // Called on initialization (once)
    @Override
    //@SuppressWarnings("all")
    public void runOpMode()
    {
        //Make sure that the robot components are found and initialized correctly.
        //This all happens during init()
        /*************************** DRIVING MOTORS ***************************/
        right = Initialize(DcMotor.class, "right");
        left = Initialize(DcMotor.class, "left");

        //Reverse the opposite side of the motors.
        if (left != null)
            left.setDirection(DcMotor.Direction.REVERSE);

        /*************************** OTHER MOTORS ***************************/
        pusher = Initialize(DcMotor.class, "pusher");
        harvester = Initialize(DcMotor.class, "harvester");

        //NOTE: Actually attempting to use null motors will cause the program to terminate.
        //This advanced system is designed for when only specific hardware is required.
        //This code should tell you which motors and sensors are not configured before the program starts running.
        //Kudos Makiah

        //Actual program thread
        try {

            //Custom Initialization steps.
            driverStationSaysINITIALIZE();

            //Wait for the start button to be pressed.
            waitForStart();

            driverStationSaysGO(); //This is where the child classes differ.
        }
        //In case the driver station says that the program has to end immediately.
        catch (InterruptedException e)
        {
            //For some reason, this causes the app to restart.  Have to look into this.  Check SuppressWarnings maybe?
            StopPlayingAudio(); // HAS TO BE FIRST LINE, otherwise this stops later on than it is supposed to.
            OutputToDriverStation("Driver Station says STOP!");
            driverStationSaysSTOP();
            //Thread.currentThread().interrupt();
        }
    }

    //Optional overload.
    protected void driverStationSaysINITIALIZE() throws InterruptedException {}
    //Has to be implemented.
    protected abstract void driverStationSaysGO() throws InterruptedException;
    //Has to be implemented.
    protected abstract void driverStationSaysSTOP(); //Can't throw InterruptedExceptions, this is why this is here.


    /*** USE TO PLAY MUSIC, AND FOR DEBUGGING PURPOSES ***/
    protected enum DownloadedSongs
    {
        JOHN_CENA_INTRO,
        MISSION_IMPOSSIBLE,
        RUSSIAN_NATIONAL_ANTHEM
    }

    private MediaPlayer mediaPlayer = null;
    protected void PlayAudio(DownloadedSongs choice)
    {
        if (mediaPlayer == null)
        {
            try
            {
                int selectedSong = com.qualcomm.ftcrobotcontroller.R.raw.jcena;
                switch (choice)
                {
                    case JOHN_CENA_INTRO:
                        selectedSong = com.qualcomm.ftcrobotcontroller.R.raw.jcena;
                        break;
                    case MISSION_IMPOSSIBLE:
                        selectedSong = com.qualcomm.ftcrobotcontroller.R.raw.missionimpossible;
                        break;
                    case RUSSIAN_NATIONAL_ANTHEM:
                        selectedSong = com.qualcomm.ftcrobotcontroller.R.raw.nationalanthem;
                        break;
                }
                mediaPlayer = MediaPlayer.create(hardwareMap.appContext, selectedSong);
                mediaPlayer.start();
                mediaPlayer.setOnCompletionListener(new MediaPlayer.OnCompletionListener()
                {
                    public void onCompletion(MediaPlayer mediaPlayer1)
                    {
                        mediaPlayer1.release();
                    }
                });

                OutputToDriverStation("Playing " + choice.toString());

                sleep(1000); //Give the MediaPlayer some time to initialize, and register that a song is being played.
            } catch (Exception e)
            {
                OutputToDriverStation("Error when attempting to play music.");
                return;
            }
        }
        else
        {
            OutputToDriverStation("Cannot play " + choice.toString() + ": already playing music.");
        }
    }

    protected boolean CurrentlyPlayingAudio()
    {
        return mediaPlayer != null;
    }

    //Used to make the media player stop playing audio, and also to prevent excess memory allocation from being taken up.
    protected void StopPlayingAudio()
    {
        if (mediaPlayer != null)
        {
            if (mediaPlayer.isPlaying())
                mediaPlayer.stop(); //stop playing
            mediaPlayer.release(); //prevent resource allocation
            mediaPlayer = null; //nullify the reference.
            OutputToDriverStation("Released Media Player");
        }
    }

    /*** USE TO OUTPUT DATA IN A SLIGHTLY BETTER WAY THAT LINEAR OP MODES HAVE TO ***/
    ArrayList<String> linesAccessible = new ArrayList<>();
    private int maxLines = 7;
    protected void OutputToDriverStation(String newLine)
    {
        //Add new line at beginning of the lines.
        linesAccessible.add(0, newLine);
        //If there is more than 5 lines there, remove one.
        if (linesAccessible.size() > maxLines)
            linesAccessible.remove(maxLines);

        //Output every line in order.
        telemetry.update(); //Empty the output
        for (String s : linesAccessible)
            telemetry.addLine(s); //add all lines
        telemetry.update(); //update the output with the added lines.
    }

    //Allows for more robust output of actual data instead of line by line.  Used for driving and turning.
    protected void OutputRealTimeData (String[] data)
    {
        telemetry.update();
        for (String s : data)
            telemetry.addLine(s);
        telemetry.update();
    }
}