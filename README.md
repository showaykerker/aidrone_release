[![](https://github.com/showaykerker/aidrone_release/blob/master/assets/NCRL_Logo.png)](http://ncrl.nctu.edu.tw)

# aidrone_release


## Prerequisition
1. [Install ros-kinetic](http://wiki.ros.org/kinetic/Installation) if you haven't. (Newer version should be compatible.)
2. Install required ROS packages
	* `$ sudo apt-get install ros-kinetic-mav*`
3. [Install Catkin Command Line Tool](https://catkin-tools.readthedocs.io/en/latest/installing.html) if you haven't.
	* It is highly recommanded to use **catkin command line tool** rather than the one using in the tutorial that build workspace with catkin_make.
4. Python Packages
	* `$ pip3 install cvxopt opencv-python rospkg pyyaml empy`
	* Please let me know if I missed anything.


## Setup
1. Choose a version of rotors-simulator (`original` or `Customed`) and follow instructions to install. 
	* Origin: [ETHZ/rotors-simulator](https://github.com/ethz-asl/rotors_simulator)
	* Customed: [showaykerker/rotors-simulator](https://github.com/showaykerker/rotors_simulator)
		* Differences from the [ETHZ/rotors-simulator](https://github.com/ethz-asl/rotors_simulator):
			* Images' size are set to RGB, (120, 192, 3). Following steps list [here](https://github.com/showaykerker/smart_drone/blob/master/README.md#enable-rgb-camera).
			* Add a collision inspector sensor plugin on multirotors. See [here](https://github.com/showaykerker/rotors_simulator/blob/master/rotors_description/urdf/multirotor_base.xacro#L147) and [here](https://github.com/showaykerker/rotors_simulator/blob/master/rotors_description/urdf/component_snippets.xacro#L783).
			* Change target git repositary of [rosinstalls](https://github.com/showaykerker/rotors_simulator/blob/master/rotors_hil.rosinstall).		
	* Touble Shooting
		* [Some bugs you might encountered when building RotorS and how to solve it](https://www.twblogs.net/a/5c9f841bbd9eee5b1a06816d).
		* Other bugs that might occasionally happen and how to solve it is listed in the [Trouble Shooting](https://github.com/showaykerker/aidrone_release#trouble-shooting) part below.
2. Try if this command works. `$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic`
	* If not, find it in trouble shooting or sent a pull request with solution in Trouble Shooting.
3. Fork this repositary or just clone this repositary into `[catkin_ws]/src/`.
4. Run `$ catkin build` in `[catkin_ws]/`.
5. ```$ chmod +x [Every node files under aidron_release/]```.
6. Changed interpreter path in the first line of every nodes.


## Usage
* $ `roslaunch aidrone_release empty_world.launch`
* $ `rosrun aidrone_release basic_usage`


## Trouble Shooting
### Installation Problems

1. `ImportError: No module named defusedxml.xmlrpc`
	* `$ pip install defusedxml`
	* [reference](https://answers.ros.org/question/260377/no-module-named-defusedxmlxmlrpc/)

### Other Troubles

1. Remove `ros-cv2` package
	* `sudo mv /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so /opt/ros/kinetic/lib/python2.7/dist-packages/cv2_bkp.so`
2. `[Error] bad callback`
	* Modify the code in `/opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py`
		* `$ sudo nano /opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py`
		* Modify `for vv in itertools.product(*[zip(*s)[0] for s in stamps]):` to `for vv in itertools.product(*[list(zip(*s))[0] for s in stamps]):`
	* Error Message
``` python
[ERROR] [1571199903.423517, 1181.760000]: bad callback: <bound method Subscriber.callback of <message_filters.Subscriber object at 0x7f0a1aa23668>>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
	cb(msg)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py", line 75, in callback
	self.signalMessage(msg)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py", line 57, in signalMessage
	cb(*(msg + args))
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py", line 282, in add
	for vv in itertools.product(*[zip(*s)[0] for s in stamps]):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py", line 282, in <listcomp>
	for vv in itertools.product(*[zip(*s)[0] for s in stamps]):
TypeError: 'zip' object is not subscriptable
```
3. `[Error] bad callback`
	* Modify the code in `/opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py`
		* `$ sudo nano /opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py`
		* Add `from functools import reduce` to the file.
	* Error Message
``` python
[ERROR] [1578127199.382906, 22102.660000]: bad callback: <bound method Subscriber.callback of <message_filters.Subscriber object at 0x7f19678dfb00>>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
	cb(msg)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py", line 75, in callback
	self.signalMessage(msg)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py", line 57, in signalMessage
	cb(*(msg + args))
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py", line 220, in add
	common = reduce(set.intersection, [set(q) for q in self.queues])
NameError: name 'reduce' is not defined

```

4. `mav_msgs` version issue
	* Caused by version mismatch of px4 and client. To solve the issue, following the instruction below.
		* If both `mav_msgs` packages were installed using `$ sudo apt-get install ros-kinetic-mav-msgs`:
			* It needs to remove one of the package. `$ sudo apt-get remove ros-kinetic-mav-msgs`.
			* Build it from source, check [this repositary](https://github.com/ethz-asl/mav_comm).
			* Do the rest below.
		* Check the version of `mav_msgs` on both server and client.
			* `$ roscd mav_msgs`
			* `$ cat package.xml | grep "<version>" `
		* `mav_msgs` package is tracked by git, just rebase one of the repositary on the tag of another one's version and rebuild it.
			* `$ roscd mav_msgs`
			* `$ git checkout [version_tag]`
			* `$ cd [catkin_ws_path]`
			* `$ catkin build mav_msgs`
```
Client [/rostopic_5927_1502794795953] wants topic /mavros/state to have datatype/md5sum [mavros_msgs/State/9e3d873fae342c8f48a8bd64c53d991e], but our version has [mavros_msgs/State/63d9a29b6abd9e0beeba06109556d062]. Dropping connection.
```