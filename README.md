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
This is one of our "basic" methods and was the first created following bump stop. It utilizes the time module in python and a known angular speed of pi/10 to drive almost exactly one circle over the span of 20 seconds. This method was a good starting point as it furthur introduced us to publishing and subscribing, specifically with Twist for movement, as well as timing actions using the time module.

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
This was the first of the three "advanced" behaviors we completed and was loosely based off the wall follower code from Charlie and Christopher's project. At this point we had used the lidar scanner, through the '/scan' topic and 'LaserScan' message type, only in the ranged stop method during class, so this was our first real look into using it. 

The data received from scan was in the form of a '1 by 361' array with each indice relating to the distance from the scanner at one degree out of the 360 degree scan. We then organized this data using 'numpy', imported as 'np', and another array with integers in a range from 1-361. Using 'np.where', we first removed the distances either below 0.1 meters or above 2 meters as these are often either the neato itself or data to far away to be useful. Also with 'np.where' organized the data into eight groupings:

'front_left', 'front_right'
'left', 'left_front', 'left_back'
'right', 'right_front', 'right_back'

With the '/scan' data organized we were then able to begin the logic behind the wall follower. This begins by determining which wall is closest between the left and right, which we find by comparing the means of the 'left' and 'right' sections of the distance array. Once the neato has determined which was it is closer it also compares the means of the front and back sections of the corresponding side. If the mean of the 'left_front' section and that of the 'right_front' section are not within a certain threshold of closeness, the neato isn't aligned parallel to the wall, and slight angular velocity is added to compensate for that difference. Along with this, the neato also checks to make sure the front and back sections aren't too far apart, as this often means the wall isn't long enough to cover both sections, which was something we found could cause big issues. An example of our velocity calculation code for the left side can be seen below:

```Python
elif self.lmean < self.rmean:
    if abs(self.lfmean - self.lbmean) < 0.01 or abs(self.lfmean - self.lbmean) > 0.3:
        msg.angular.z = 0.0
    else:
        msg.angular.z = float(
            self.lfmean - self.lbmean) * self.ang_vel
    self.vel_pub.publish(msg)
```

The one other aspect of the wall follower we added later on was the ability to switch walls if presented with a frontal obstruction (generally a wall). To complete this, we checked whether either the 'front_left' or 'front_right' sections had a smaller, and therefore closer, average distance than either left or right. In this instance we would use whether we were closer to the left or right wall to determine which way to turn. Once determined, using the same speed as in our circle behavior, a velocity of either '-pi/10' or 'pi/10' was applied for 5 seconds, resulting in a 90 degree turn. The neato would then continue following this new wall until a new obstruction appears. 

### Code Structure
Despite being much more complex, the code only contained one more function which was used to receive and process scanner data as it was received by the subscriber.

```Python
class CircleNode(Node):
    def __init__(self):
        # code
    def run_loop(self):
        # code
    def process_scan(self, data):
        # code
def main(args=None):
    # code
```

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



