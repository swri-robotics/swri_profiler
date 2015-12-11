swri_profiler
=============

Swri_profiler is a lightweight profiling framework for C++ ROS nodes.
It allows you to selectively measure how much time is spent in various
scopes.  Profiling data is generated and published to a ROS topic
where it can be recorded or monitored in real time.  The profiler was
designed to be lightweight enough that it can be left in during normal
operation so that performance data can be monitored at any time.



Getting Started
===============

Add swri_profiler to your ROS workspace, update the workspace to
clone your repository, and build swri_profiler with catkin.

Once built, you can test that everything was installed properly with
the built in test nodes:

1. Start a roscore

   You can skip this step, but it's convenient to have a separate
   roscore running so you can start and stop the other launch files
   easily.

2. Launch swri_profiler / testing_example.launch
  
   As soon as the nodes are running, the profiler will begin
   collecting profiling data and publishing it.  You can verify that
   the profiler is working by watching the /profiler/data and
   /profiler/index topics.


3. Launch swri_profiler / profiler.launch

   The profile viewer is implemented as a webpage with javascript.
   This launch file starts up roslibjs rosbridge_server and runs a
   lightweight Python server to server the webpage.  (**Be aware that
   the Python server will serve pages to any clients.**)

4. Point a web browser to http://localhost:8000/

   The viewer is useful but still in a very preliminary state.  If you
   see some colorful but boring plots this point, that means
   everything is working.  


Using the profiler
==================

Adding the profiler to a node is very straightforward:

1. Add swri_profiler as a dependency to your package. 

2. Include swri_profiler/profiler.h in files that you want to profile.

3. Call the macro SWRI_PROFILE("my-label") at the start of any scope
   that you want to profile.

SWRI_PROFILE creates a local variable with a unique name at the point
where it is called.  The local variable records the time when it was
created.  When the variable goes out of scope, the end time is
recorded to get the running time of your code.  

That's all it takes to get started.  The profiler will automatically
initialize itself when it is first used, and automatically close
itself when the ROS node is shutdown.


Profile Stack
=============

The profiler is aware of all currently running profiles.  When a new
profile is started, it becomes a child of the profile that it is
running in.  This allows the profiler to report the results as a call
tree.  The tree is defined by where you place calls to SWRI_PROFILE
rather than the actual call stack.  This allows you to selectively
collect as detailed or broad of a profile as you want.

Consider this simple example:

```
void handleOdometry(...)
{
    SWRI_PROFILE("handle-odometry");
    
    runStateEstimator();
    publishOutput();
}

void runStateEstimator(...)
{
    SWRI_PROFILE("run-state-estimator");
    /* do some work... */
}

void publishOutput(...)
{
    SWRI_PROFILE("publish-output");
    /* do some work... */
}
```

This code will result in three distinct items in the profiler:

* handle-odometry
* handle-odometry/run-state-estimator
* handle-odometry/publish-output

The profiler viewer can use this data to estimate the inclusive and
exclusive time spent in each block.  This is extremely convenient
because the tree is generated automatically without any extra work
from you.


Tips
====

1. Like ROS, Swri_profiler uses the forward slash "/" to distinguish
namespaces.  Avoid using "/" in your profiler labels unless you know
what you're doing.  The profiler won't care, but it can cause
confusing output.

2. Add a profile to every ROS callback in your node:

```
SWRI_PROFILE(ros::this_node::getName());
```

Tracking every callback allows you to get a complete picture of how
much time is spent in the node, and allows you to compare the time
spent between different nodes.  The name returned by ROS will include
the nodes namespace, allowing you to also measure how much
(instrumented) time is spent in a given namespace and compare the
loading between namespaces.

If you also want to track a callback individually, just tack on
another SWRI_PROFILE:

```
SWRI_PROFILE(ros::this_node::getName());
SWRI_PROFILE("callback-label");
```




