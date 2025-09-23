# Comprobo Warmup Project
*ENGR3590: A Computational Introduction to Robotics*

*Grant Rechtin*\
*Brenna O'Donnell*
-----------------------------------------------------------------------------------------------------------
The goal of this project is to become familier with Ros2, the Neatos, and the systems around them that we will be using throughout this course. We started with basic driving to gather an understanding of subscribers and publishers but through this project have learned to use the lidar scanner, bump sensor, and Twist movement commands to create both teleop and autonomous movement. These behaviors were what we completed by the end.

- [Teleop Drive](#teleop-drive)
- [Drive in Circle](#drive-in-circle)
- [Wall Follower](#wall-follower)
- [Person Follower](#person-follower)
- [Turn Around](#turn-around)
- [Spiral](#spiral)
- [Finite-State Controller](#finite-state-controller)

## Teleoperation  <a name="teleoperation"></a>
### Description
### Methods
### Code Structure

## Drive in Circle  <a name="drive-in-circle"></a>
### Description
Drives in a circle once over the span of 20 seconds.

### Methods
This is one of our basic methods and was the first created following bump stop. It utilizes the time module in python and a known angular speed of pi/10 to drive almost exactly one circle over the span of 20 seconds. This method was a good starting point as it furthur introduced us to publishing and subscribing, specifically with Twist for movement, as well as timing actions using the time module.

### Code Structure
As this was our first real behavior, and it was an incredibly simple one, the node only contained an init and run_loop.

```Python
class CircleNode(Node):
    def __init__(self):
        # code
    def run_loop(self):
        # code
def main(args=None):
    # code
```

## Wall Follower  <a name="wall-follower"></a>
### Description
Follows and aligns with the nearest wall on either the left or right side until front in blocked, in which case it turns in the unblocked direction.

### Methods


### Code Structure

## Person Follower  <a name="person-follower"></a>
### Description
### Methods
### Code Structure

## Turn Around  <a name="turn-around"></a>
### Description
### Methods
### Code Structure

## Spiral  <a name="spiral"></a>
### Description
### Methods
### Code Structure

## Finite-State Controller  <a name="finite-state-controller"></a>
### Description
### Methods
### Code Structure


## Conclusion
### Challenges
### Improvements
### Take-aways



