# Programming team learning resources

Resources for team members wanting to learn to program our bot, vision subsystem, LEDs, and more.

## Tools

The tools we use for programming are:

* Visual Studio Code &mdash; Microsoft's free, open source code editor is now the preferred programming tool for FIRST teams. We will be using it in the 2018-2019 season
* GitHub &mdash; We store all our source code in GitHub. This makes it easily available to every member. Plus, by publishing it with an open source license, by FIRST rules we can re-use code as well as use code published by other teams.

### GitHub

* [GitHub learning lab](https://lab.github.com/)

### Visual Studio Code

* [Microsoft's intro tutorial](https://code.visualstudio.com/docs/introvideos/basics)


## Programming languages

We use:

* Java &mdash; to program the bot
    * WPLib &mdash; is a Java add-on library that we use to control the bot
* Python &mdash; for computer vision, target identification, etc.
    * OpenCV &mdash; is a library for image processing, computer vision, etc. which is available for Python (and other languages)
* Arduino/C++ &mdash; for controlling the LED "blinkies"

### Java

Java is the primary programming language we use for controlling the bot. 

* [Free Udemy Java course](https://www.udemy.com/java-tutorial/)
* [Learn Java Online](https://www.learnjavaonline.org/)
* [Codecademy](https://www.codecademy.com/learn/learn-java) (just skip the paid lessons)


##### WPILib

Wooster Polytechnic provides the primary robot control library.

* [Screensteps live](http://wpilib.screenstepslive.com/s/currentCS) 
* [WPI library API docs](http://first.wpi.edu/FRC/roborio/release/docs/java/)

### Python

Python is what we use on the Jetson / Raspberry Pi for computer vision, target identification, etc. 

##### Installing Python

We use python 3.x (not the older v2.7 that comes installed on some systems). Make sure you install and use the correct version.

The easiest, perhaps best way to get a good python installation with all the tools you need is to install [Anaconda](https://www.anaconda.com/download/).

(For a non-Anaconda way to install python, see https://www.python.org/)

Anaconda includes a couple of tools that will make writing python apps convenient:

* Jupyter Notebooks - offers you a way to write and run python code in a browser window and see the output in that window. You can then share your resulting project as a runnable "notebook" for others to use.
* Visual Studio Code - you can optionally install VSCode as part of Anaconda, pre-configured to work with Python projects


##### Learning Python

* [How to code in Python 3](https://www.digitalocean.com/community/tutorials/digitalocean-ebook-how-to-code-in-python) A free e-book (epub & PDF formats) from Digital Ocean (also available in [WorldCat](https://www.worldcat.org/title/how-to-code-in-python-3/oclc/1020289950)
* [LearnPython.org](https://www.learnpython.org/) - a basic tutorial. Note that this is an older web site and none of their interactive code snippets seem to run any more. But you can run the code locally.
* [Codecademy](https://www.codecademy.com/learn/learn-python) - free python course (just skip over the paid modules)
* [Codeschool](https://www.codeschool.com/courses/try-python) - free python course
* [Udacity Programming Foundations with python](https://www.udacity.com/course/programming-foundations-with-python--ud036) - free video-based course
* [Google's python tutorial](https://developers.google.com/edu/python/) - not the snazziest of tutorials, but covers the basics

#### OpenCV

OpenCV is a great open source computer vision library. It is a big and complex library that can be hard to navigate. There are OpenCV versions for Python, C/C++, Java & Android, and iOS.

*We suggest you wait to explore OpenCV until you have a good feel for programming in Python.*

Some learning resources:

* [Team 254's presentation](https://www.team254.com/documents/vision-control/) on computer vision and motion control (worth watching to get an idea of what can be done as well as how to do it)
* [OpenCV-Python Tutorials](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html)
* [PyImageSearch](https://pyimagesearch.com)
* [LearnOpenCV.com](https://learnopencv.com)
* [Tim Poulsen's blog](https://timpoulsen.com)

## Arduino / C++

For our LED strips, we use an Arduino controller and the C++ programming language. 

* [Our LED repo](https://github.com/Raider-Robotics-Team-1518/LED) covers the basics of how our LEDs and Arduino are setup as well as the code we've used in the past
* [Adafruit's tutorials](http://www.ladyada.net/learn/arduino/) Adafruit is a supplier of Arduino boards, compatibles, and components. They offer a good selection of tutorials

## Looking ahead

### RobotPy 

[RobotPy WPILib](http://robotpy.readthedocs.io/en/stable/getting_started.html) is a set of libraries that are used on your roboRIO to enable you to use Python as your main programming language for FIRST Robotics robot development. It includes support for almost all components that are supported by WPILib’s Java implementation. We are not currently using this, but are considering it so that we can focus on a single programming language for almost everything we do.

