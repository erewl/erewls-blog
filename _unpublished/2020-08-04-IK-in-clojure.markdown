---
layout: post
title:  "Simple Inverse Kinematics with Clojure"
date:   2020-08-04 13:55:55 +0200
categories: clojure quill drawing
---

# A simple Inverse Kinematics algorithm in Clojure

Inverse Kinematics is a process used a lot e.g. in computer animation and robotics.
It is used to calculate the endeffector of a series of connected segments.
To be more precise: How would a robotic arm have to move its joints to reach Point (X|Y|Z)? The Point (X|Y|Z) in that example is the endeffector that will be calculated with Inverse Kinematics (IK).

While searching for a good explanation how the math behind Inverse Kinemtics is working, I have found [this](https://www.youtube.com/watch?v=UNoX65PRehA) video explaining the FABRIK-algorithm. FABRIK is short for **F**orwards **A**nd **B**ackwards **R**eaching **I**nverse **K**inematics. It is not a 100% precise algorithm but rather approximates the endeffector.

Before diving into Clojure, I explain in my own words what an endeffector is.

One segment of an robotic or animated arm consists out of the point (m<sub>x</sub>|m<sub>y</sub>) and its segment length `r`.
The arm's reach is constrained by its length r and could reach any points on the circle (e.g. (x, y) in the gif below).

<img src="../pictures/ik/simple_segment.gif" alt="drawing" width="300"/>


This is the smallest building piece and can be used to build more complex robotic arms with multiple joints:

![](../pictures/ik/two_segments.png)

Calculating the endeffector of a complex robotic arm precisely can be difficult. In most cases an approximation to the endeffector can be sufficient. And here is where the FABRIK-algorithm sets in.

The algorithm starts with the backwards reach calculation:

TODO backwards and forwards mathematical explanation



Given are the coordinates (x,y) for all segments and the position (ex, ey) of the endeffector.
Let's say you have three segments S1, S2 and S3, each described in clojure as two coordinates `x` and `y` and the reach `r`, e.g.:
The segments or the arm you can describe as an array of those segments:

```clojure
(def arm [{:x 2 :y 0 :r 50}
          {:x 52 :y 0 :r 20}
          {:x 72 :y 0 :r 10}])
```

To display the robotic arm I used quill's `q/line` function. To get the start and end coordinates of one segment I partitioned my array:

```clojure
(let [segments (partition 2 1 arm)]
    (doall (map (fn [[a b]] (q/line (:x a) (:x b) (:y a) (:y b))))))
```


For the process of building a vector between two points, normalizing it and bringing it up to a desired length I have created those helper functions:
```clojure
(defn getNormalVector
  "normalizes the vector between points a and b"
  [a b]
  (let [x (- (:x b) (:x a))
        y (- (:y b) (:y a))
        length (Math/sqrt (+ (* x x) (* y y)))]
    {:x (/ x length)
     :y (/ y length)}))

(def createSegment 
    "creates a segment from a vector and reach"
    [vector reach]
        {:x (* reach (:x vector))
        :y (* reach (:y vector))
        :r reach})
```

For the backwards reach the arm array can be reversed:
```clojure
(let [backwards (reverse arm)
segments (partition 2 1 backwards)]
    ...)
```

```clojure
(defn simpleInit
  "initialises a chain of segments as a straight line"
  [lengths startX startY]
  {:points (reduce (fn [acc curr]
                     (let [prev (peek acc)]
                       (conj acc {:x (+ curr (:x prev)) :y (:y prev) :r curr})))
                   [{:x startX
                     :y startY
                     :r (first lengths)}]
                   lengths)})
```

```clojure
(defn calculateReach [points endPoint]
  (let [endPointWithSegmentLength (conj endPoint {:r (:r (first points))})
        parts (seq (partition 2 1 points))]
    (reduce
     (fn [acc curr]
       (let [[a b] curr
             prevPoint (peek acc)
             length (:r a)
             direction (getVectorOfLength prevPoint b length)]
         (conj acc {:x (+ (:x direction) (:x prevPoint))
                    :y (+ (:y direction) (:y prevPoint))
                    :r length})))
     [endPointWithSegmentLength]
     parts)))
```

```clojure
(defn calculationIK [arm endEffector]
  (let [points (:points arm)
        originalStartPoint (first points)
        backwardsReach (calculateReach (reverse points) endEffector)
        forwardsReach (calculateReach (reverse backwardsReach) originalStartPoint)]
    (assoc arm :points forwardsReach)))
```