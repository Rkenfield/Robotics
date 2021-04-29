Instructions for running final project:

	Ryans Contributions:
		In the end our stuff never got implemented together but we have seperate levels to show what we did for our individual contributions.

		To run/see Ryans contributed code 
		
			-open up the worlds file and open RyanContributionsLevel.wbt in webots

			-open up the HiderController and the TempSeekerController and ensure 
			
			-in the HiderController make sure the state variable at line 27 is set to "Explore"

			-in the TempSeekerController make sure that the state variable at line 21 is set to "seek"

			-then you should be able to hit play and it should run fine. It may be slow to watch at normal speed so feel free to fast forward

				-The epuck that starts in the middle is the Hider and the one that starts on the left is the Seeker

				-The Hider and Seeker will run for a bit while the Hider maps the level you can see what it maps and its location on the display when the epuck switches to the path finder state it will show a plot of the map matrix
				close this window and the program will pause while it finds a path to the goal point depending on the epucks distance from the point it might take some time to find a path but when it does find a path the path will be drawn on the display
			
				-Then the epuck will go into path follower state on its own the hider has trouble following the path it generates due to some issues listed in the report, also if the seeker gets too close to the hider the hider will go back to explorer mode for a bit and have to redo its steps

			-The program is done when the console prints out Hider has exited

			-PS. If you don't want to see the Seeker epuck interupt the Hider epucks behaviour feel free to drag the epuck that starts on the left out of the arena and set the variable in TempSeekerController at line 21 to anything


Caitlyn's Contributions:

We ran out of time to implement our levels together. However, we have separate levels to show our individual work.
To run my code:
	- open up the worlds file and open CaitlynsLevel.wbt in Webots
	- open up the mavic2pro.py controller
	- hit play
	- since the keyboard commands sometimes work, you can test them using your keyboard. make sure to click in the 3D window to gain access

If it is running correctly, there will be a "Starting the drone..." message in the console, followed by a menu with the options for keyboard commands. Again, to use these please make sure to click in the 3D window. Without key commands received, the drone will fly up, then down again within the map. It will then fly upwards again, then down to the beam exposed out of the map, but it will ascend before hitting it. If you zoom out more, you will see that the drone continues to fly from the map in an upward wave pattern.