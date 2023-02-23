# Description

The iri_dynamic_transform_publisher package broadcasts a tf transform between two frames as defined with dynamic reconfigure parameters. 

Useful for dynamically adjust and find a desired static transform, to be replaced once found by a tf/static_transform_publisher node


# Dependencies

This node has the following dependencies:

 * [iri_base_algorithm](https://gitlab.iri.upc.edu/labrobotica/ros/iri_core/iri_base_algorithm)

 * [tf](http://wiki.ros.org/tf)

# Install

This package, as well as all IRI dependencies, can be installed by cloning the 
repository inside the active workspace:

```
roscd
cd ../src
git clone <url-to>/iri_dynamic_transform_publisher.git
```

However, this package is normally used as part of a wider installation (i.e. a 
robot, an experiment or a demosntration) which will normally include a complete 
rosinstall file to be used with the [wstool](http://wiki.ros.org/wstool) tool.

# How to use it

The *node.launch* file is intended to be included anywhere needed.

The *test.launch* file shows an example of usage, including node.launch, running rqt_reconfigure to adjust the transform parameters, and showing the transform on Rviz.