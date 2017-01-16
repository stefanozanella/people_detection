## Abstract
This paper describes a people detection algorithm for three-dimensional point
clouds based on the well-known Viola-Jones face recognition algorithm (Viola and
Jones, 2003). The key contribution of the project is an implementation of
mentioned algorithm, adapted for work with 3D point clouds. The implementation
consists in a cascaded classifier and its training algorithm. This detector
serves as a cornerstone for the overall people detection algorithm, which
features a variant of the region-growing segmentation algorithm found in the PCL
library. Analysis of the algorithm's performance are provided based on a
given set of 40 images.

## Introduction

This paper attempts at extending the current research in the field of
recognition of human beings in three-dimensional point clouds. The key intuition
behind the proposed approach is that human presence in a scene might be
connected to the presence of any part of a human body.

Building a detector flexible enough to recognize any body part (e.g. a face or a
hand), or any sensible combination of them (e.g. a leg and a foot), is a task
that has low chances to lead to good performances. Indeed, also designing a
detector that recognizes bodies in multiple poses is not a simple task, as the
flexibility of human body makes the number of possible poses very high.
Moreover, such a detector might not perform well in case of obstructed bodies or
bodies only partially framed into the scene.

Therefore, the proposed approach starts from the most distinguishing part of
a human body: its head. A human head, put in the context of a larger scene,
provides some desirable features:

* it's relatively rigid, meaning that it acts approximately as any other rigid
  body that can only be subject to rotation. This is important as it allows to
  be detected by using standard object recognition techniques.

* it's small compared to the rest of the body, opening the door to algorithms
  that achieve higher performances than those than need to perform larger scans
  of a scene or analyze larger sets of points at a single time.

* it's possibly the body part that's most discriminative of human nature: e.g.
  it might be hard, at a relative distance, to say if a barefoot belongs to a
  dummy; on the other hand, it's definitely easier if a frontal head it's human
  or not. This might be of help to recognition algorithms.

The subsequent reasoning step is that, given an accurate way of detecting a
human head in a scene, we can extend the detection to points that seems to
belong to the same object as the head. This is a task that segmentation
algorithms can solve pretty well, with a good range of choices in terms of
speed, accuracy and performances.

This sort of "detection expansion" technique offers a few advantages:

* it allows to detect bodies that are partially obstructed or not entirely in
  the scene
* it makes the algorithm robust to changes of pose, given the segmentation
  algorithm can correctly identify "compact" objects (i.e. clusters of point
  that are mostly equidistant from each other).
* if decomposes a "hard" problem into two problems that has been alredy well
  treated in literature, and for which a number of efficient solutions exist.

As a matter of implementation, two well-known algorithms have been selected:

* for head (face) detection, the algorithm devised by Viola and Jones (Viola and
  Jones 2003) has been selected and implemented with a number of adaptations and
  improvements made possible by its application in the 3D space.
* for the body detection, a variant of the region-growing segmentation algorithm
  present in the PCL library has been used

### Overview

The remaining of this paper will describe more in detail the face detection
algorithm implementation and its peculiarities. We will start by describing the
face detection algorithm and all the specific changes that have been introduced
for it to be applicable to the 3D space. After that the text will focus on the
segmentation algorithm. Finally the results of running the algorithm described
on a validation set of 40 images will be discussed. A selection of ideas for
future improvements closes the paper.

## Face detection

## Body segmentation

## Results

## Further developments

* better algorithm training to remove the need of additional processing
* different measurements can be used to calculate features (e.g. face depth map)
* parallelization of feature values calculation for even faster implementation
* combination of multiple, parallel detectors to improve detection performance
  and allow a wider range of poses to be identified (e.g. profile detector)
* use of color information to improve detection performance (human skin lies in
  a restricted range of color variations)
* adaptation of the algorithm to take advantage of moving targets (i.e.
  implementation of a companion tracking algorithm)
