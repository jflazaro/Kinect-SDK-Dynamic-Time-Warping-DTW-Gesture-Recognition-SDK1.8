---------------------------------------------------------------------
KinectDTW - Kinect SDK Dynamic Time Warping (DTW) Gesture Recognition
---------------------------------------------------------------------
By Rhemyst
Implementation by Rymix
Copyright (c) 2011, Rymix
http://kinectdtw.codeplex.com/


Notices
-------
This source code is distributed freely as open source. We'd rather you kept 
this message in tact and included our copyright headers in your project. 

Microsoft's SDK is not for commercial use, so by extension neither is this 
library. Always adhere to Microsoft's terms of Kinect SDK use: 
http://research.microsoft.com/en-us/um/legal/kinectsdk-tou_noncommercial.htm

No warranty or support given. No guarantees this will work or meet your needs. 
Some elements of this project have been tailored to the authors' needs and 
therefore don't necessarily follow best practice. Subsequent releases of this 
project will (probably) not be compatible with different versions, so whatever
you do, don't overwrite your implementation with any new releases of this 
project!


Release information
-------------------
Release version 0.1
Full of bugs, bad practice and not flexible enough yet, but a good starting 
place for the community to build upon.


Introduction
------------
The Kinect SDK is rocking a whole bag of awesome, of that there's little doubt,
but until the open source community make some of its most tasty stuff 
compatible with the SDK, Microsoft's offering will remain a little 
under-featured and under-developed.

Amongst the missing features is a gesture recognition system. Lots of people 
are building their own, bespoke implementations but no package (that I've 
seen, at any rate) providesfast, accurate and programmable gesture libraries.

That's where this project could help. Using a relatively novel approach, 
KinectDTW uses a vector-based nearest neighbour algorithm to track and classify
your gestures. The Dynamic Time Warping element means that this can recognise 
gestures performed at different speeds.


Instructions
------------
1. Open up the Solution and check that you have all the prerequisite 
software/dlls. Put Visual Studio into Debug mode and build and run the project.
You should see the MainWindow XAML window appear. If anything goes wrong at 
this point, check your references, any paths that might be wrong or any other 
dependencies that I might have forgotten.

2. Step into view of the Kinect sensor. Your skeleton should be tracked almost 
immediately. It's probably best is you do this with only you in Kinect's sight 
as this release only deals with one player.

3. Load the sample gestures by clicking Load gesture file and navigating to 
the supplied RecordedGestures<date>.txt file

4. Start performing some gestures. You can see the names of the gestures from 
the select box, and hopefully most of them are obvious to perform. You will 
see matches appear in the results text panel at the top of the screen.

5. Nuke the app and start again. 

6. Try recording your own gestures. Make sure your skeleton is being tracked, 
select the gesture name you want to record, then click the Capture button. 
You have three seconds to get into place and start recording your gesture. The 
gesture is currently hard-coded to look at 32 frames (which is actually every 
other frame over 64 expended frames). You may want to tweak this setting.

   When recording gestures it is important that you start your gesture as soon 
   as the recording starts and that you finish on the 32nd frame. This might 
   mean that you have to perform your gestures for the recording slower (or 
   perhaps more quickly) than you would do in real life. Stick with it. The 
   DTW algorithm doesn't care about how quickly the gesture is performed.

7. When recording of each gesture is finished it automatically switches back 
into Read mode, so test your new gesture a few times to see if you're happy 
with it. If not, re-record it and try again.

8. When you're happy with your results, save your gestures to file. I'd 
really love it if you shared your gestures with the community so that we can 
build up a library of reliable gestures.

9. Make your own gestures - simply amend or add to the selectbox items with a 
unique name and record your gesture. Note that a gesture name must start with 
an @

That's it for this demo. Of course, it's not production specification yet, nor 
would you want to release the gesture recorder in your project (only the 
recogniser), but hopefully this gives you a good start in producing your own 
gestures. DTW is probably not the perfect solution for all gestures, either, so
you'll need to experiment to find out what works and what doesn't. However, it 
is a powerful tool for general gesture recognition.


Contribution and community
--------------------------
(Almost) anyone is free to contribute to this project. Just get in contact via 
Codeplex.

This framework's strength could well be in a community-built libray of tried 
and tested gestures. Please contribute your gestures!


Features
--------
* 2D gesture recognition using all the joints of the upper torso (Skeleton 
Frame)

* Fast and customisible gesture recogniser

* Gesture recorder, so you can create your own gestures

* Save gestures to file for future use (and load from file)

* Sample WPF project with skeletal viewer and optional depth and RGB viewers


Future developments
-------------------
* 3D gesture recording and recognition

* More customisible parameters for finer control

* Dynamic gesture length

* Voice control for starting, stopping and switching gestures in the recorder 
(so that you don't have to keep returning to the keyboard/mouse and messing up 
your gesture recording)

* Customisible 'active joints' for gesture recognition (i.e. track just the 
right arm and hand rather than the whole upper body)

* Ability to deal with more than one player

* Smooth out the recognition of similar gestures so that only one is selected


Requirements
------------
Windows 7
Kinect SDK
Visual Studio 2010
Coding4Fun toolkit http://c4fkinect.codeplex.com/
Probably the latest .NET and maybe some other stuff too. You'll soon find out. 
Look at the project References if you're stuck.


Tested on
---------
Windows 7 Professional 32-bit SP1
Visual Studio 2010 Ultimate
Intel Pentium Dual CPU E2180 @ 2.00 GHz
4.00 GB RAM (3.25 GB usable)
ATI Radeon X1950 Pro 512MB

The SDK runs pretty slowly on my PC (which is actually beneath Microsoft's 
recommended specification for the SDK). It was unusably slow when I turned on 
the depth and RGB streams. Hopefully if you have a faster PC you'll have a 
better experience. However, the gestures are recognised very quickly.


Links
-----
KinectDTW project on Codeplex:
	http://kinectdtw.codeplex.com/

Microsot's Kinect SDK terms of use:
	http://research.microsoft.com/en-us/um/legal/kinectsdk-tou_noncommercial.htm

Wikipedia's ovbiously 100% reliable explanation of dynamic time warping:
	http://en.wikipedia.org/wiki/Dynamic_time_warping

Kinect SDK fora on MSDN:
	http://social.msdn.microsoft.com/Forums/en-US/category/kinectsdk

Microsoft Coding4Fun Toolkit
	http://c4fkinect.codeplex.com/

Disclaimer
----------
Stock disclaimer statement, but I mean it: I'm releasing this to you as a 
learning tool, nothing more. Don't expect anything from me or expect to hold 
me liable for anything whatsoever, because I herein expunge myself of all 
responsibility for this code. This software is provided by the Rymix and its 
licensors “as is” and any express or implied warranties, including, but not 
limited to, the implied warranties of merchantability and fitness for a 
particular purpose are disclaimed. In no event will Rymix or its licensors 
be liable for any direct, indirect, incidental, special, exemplary, or 
consequential damages (including, but not limited to, procurement of 
substitute goods or services; loss of use, data, or profits; or business 
interruption) however caused and on any theory of liability, whether in 
contract, strict liability, or tort (including negligence or otherwise) arising
in any way out of the use of this software, even if advised of the possibility
of such damage. 