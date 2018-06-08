# lucrezio_semantic_mapper

### Description

This ros package contains the node that manages the creation and the update of the semantic map. A semantic map is implemented as an array of objects. 
Each object has the following features:

* id: an `int` representing its unique identifier
* type: a `string` representing the semantic class of the object
* bounding_box: a pair of `Vector3f` representing the 3D bounding box
* cloud: a `PointCloud` representing the 3D model

This node subscribes to the following topics:

* /image_bounding_boxes: this message contains the 2D bounding boxes returned by the object detector
* /camera/depth/image_raw: Depth image acquired with Xtion sensor

The processing pipeline is composed of three steps:

* **Object Extraction:** takes as input the 2D bounding boxes and the depth cloud and produces the set of 3D objects currently seen by the robot

* **Data Association:** matches the objects in the current view of the robot with the ones already seen

* **Merging:** fuses the objects that have been previously associated with the ones already present in the map and adds the objects that are seen for the first time

This node publishes to the following topics:

* /visualization_markers: this message contains the 3D markers corresponding to the objects present in the map that can be visualized on RViz

### Usage

    rosrun lucrezio_semantic_mapping semantic_mapper_node

### TODO

* **Refactoring:** check if it's possible to directly subscribe to `point_cloud`
* **Test:** `data_association` and `merging`
