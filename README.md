# Programming learning resources

Resources for team members wanting to learn to program our bot, vision subsystem, LEDs, and more.

## Python

We use python 3.x (not the older v2.7 that comes installed on some systems). Make sure you install and use the correct version.

### Installing Python

The easiest, perhaps best way to get a good python installation with all the tools you need is to install [Anaconda](https://www.anaconda.com/download/).

(For a non-Anaconda way to install python, see https://www.python.org/)

Anaconda includes a couple of tools that will make writing python apps convenient:

* Jupyter Notebooks - offers you a way to write and run python code in a browser window and see the output in that window. You can then share your resulting project as a runnable "notebook" for others to use.
* Visual Studio Code - you can optionally install VSCode as part of Anaconda, pre-configured to work with Python projects
* Spyder IDE - not a VSCode fan? Anaconda also includes Spyder for writing your Python programs (a bit more like Eclipse than VSCode). 


### Learning Python

[Learning Python](Learning Python.ipynb) - a crash course in Python in the form of a Jupyter notebook included in this repo. You will need to download and run locally to get full access to this notebook:

1. Download the file (or clone this whole repo)
2. Assuming you have Anaconda installed, in the folder where the ipynb file is located, type `jupyter notebook`
3. Then, in the resulting browser window, double-click the Learning Python.ipynb file to open it
4. To run the code in a block, click the block to select it and press Shift + Enter
5. When all done, close the browser windows and press Ctrl + C in the terminal window to shut down jupyter.

[LearnPython.org](https://www.learnpython.org/) - basic tutorial, none of their interactive code snippets run, but you can run the code locally. (I think it's for python 2.7 but still covers the basics you need to know.)

[Codecademy](https://www.codecademy.com/learn/learn-python) - free python course (just skip over the paid modules)

[Codeschool](https://www.codeschool.com/courses/try-python) - free python course

[Udacity Programming Foundations with python](https://www.udacity.com/course/programming-foundations-with-python--ud036) - free video-based course

[Google's python tutorial](https://developers.google.com/edu/python/) - not the snazziest of tutorials, but covers the basics

### RobotPy 

[RobotPy WPILib](http://robotpy.readthedocs.io/en/stable/getting_started.html) is a set of libraries that are used on your roboRIO to enable you to use Python as your main programming language for FIRST Robotics robot development. It includes support for almost all components that are supported by WPILib’s Java implementation.
