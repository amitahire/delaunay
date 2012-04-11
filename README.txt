Delauny Triangulation
=====================

Building
--------

build with:

	make

to get makepoints-d and delaunay-d, or build release versions with:

	make config=release

which will result in makepoints-z and delaunay-z.

Usage
-----

First generate a point cloud:

	./makepoints-z -t uniform --seed 1234 -n 10000 --range 100 -o points.txt

Run `./makepoints-z -h` to see what these parameters mean.

Next run the triangulator. This has a bunch of options to describe the
visualization. Run `./delaunay-z --help` for a full list.

Example: skip visualization and just show the end, with a moving cutaway
	./delaunay-z --animcutaway --skip --points points.txt

Example: show visualization, with auto advancement, using space to start/stop
	./delaunay-z --animcutaway --auto --pause --points points.txt

Example: use default data set, step one operation at a time (hold space to watch it progress)
	./delaunay-z 

Add --nodebug to any of these to disable the debug primitive rendering.


Interactive Controls
--------------------

SPACE				pause or unpause if auto stepping, advance by 1 operation if not.
shift+s				large step (100 operations)
ARROW UP/DOWN		zoom camera
MOUSE BUTTON +MOVE	rotate camera


