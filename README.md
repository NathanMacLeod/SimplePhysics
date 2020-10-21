# SimplePhysics
Tried to create a simple 2d physics engine. Made around December 2018

To interact with bodies use either the force tool, force drag tool, or linear drag tool, and click on the body and drag.
New bodies can be created using the rectangle tool, N-sided polygon tool, or custom polygon tool. Toggling the "create fixed polygon" button
will make newly created bodies with a fixed position and angle. To use the create custom polygon tool drag the mouse to create the desired shape and right click to finish it.
Bodies that are interlocking or some concave shapes tend to have buggy behavior.

While the program as a whole works, some of the algorithims implmented to achieve this are a bit naaive.
For example, to calculate center of mass and inertia, it will place a LOT of points within the bounding box of the created shape
This results in less accurate calculations of these values, but also a big delay when creating large objects.

The engine approximates time of impact with intersecting bodies by iterating back and forward the two involved bodies in time until the pentration distance
is small enough to a specific margin. When writing the collision detection algorithim, I didn't follow any specific algorthim and instead had tried writing my own,
resulting in a very expensive n^2 thing that isnt that efficent. It does use some narrow phase tests first like bounding spheres and bounding boxes first though.
I went through a bit of work to try to get the collision working with concave bodies which it actually does, although in certain edge cases it will not work so well.

I actually went on to program a game using this physics engine, It is also in my github as Portal2D.

# Demonstration

Click the image below to view a demonstration of the program on youtube

[![Gameplay](https://img.youtube.com/vi/7Lbg61UJLrk/0.jpg)](https://www.youtube.com/watch?v=7Lbg61UJLrk)
