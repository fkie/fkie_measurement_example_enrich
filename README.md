# fkie_measurement_example_enrich

This package contains a simple example how to create a measurement publisher.
It is an example for the publication of the **MeasurementLocated** message ([fkie_measurement_msgs](https://github.com/fkie/fkie_measurement_msgs) package) on the Enrich.
To create this message, a measured value and the position are required.
In this example, the measured value is subscribed to a Float32 topic.

The position can be obtained from the **nav_msgs::Odometry** message or **tf**.
Odometry is used by default.
To use **tf**, the parameter **use_tf** must be set to **True**, e.g.:

```bash
ros2 run fkie_measurement_example_enrich measurement_publisher --ros-args -p use_tf:=True
```

There also two launch files provided, one for using with odometry and another for using tf.

Additional parameters can also be set, e.g. to describe the sensor or adjust the frame IDs. See the parameter descriptions in the code or launch files for more information.

You can use this example as a starting point to create your own measurement publisher, which consumes other topic types to get the measured values.

## License
This project is licensed under the MIT license.
