## Build
```
git clone git@github.com:XiandiShan/rclcpp_samples.git
cd rclcpp_samples
colcon build --cmake-clean-cache --allow-overriding demo_nodes_cpp

```

## Usage
### SimpleLifecycleNode
1. Run call_service_in_lifecycle_node
```bash
ros2 run call_service_from_lifecycle_node simple_lifecycle_node
```
2. In another terminal
```bash
ros2 run demo_nodes_cpp add_two_ints_server
```
3. Call lifecycle configure action in another terminal
```bash
ros2 lifecycle set /simple_lifecycle_node configure
```
Or, call service in another terminal
```bash
ros2 service call /simple_lifecycle_node/add_two_ints std_srvs/srv/Empty "{}"
```

### SimpleNode
1. Run call_service_in_lifecycle_node
```bash
ros2 run call_service_from_node simple_node
```
2. In another terminal
```bash
ros2 run demo_nodes_cpp add_two_ints_server
```
3. Call service in another terminal
```bash
ros2 service call /simple_node/add_two_ints std_srvs/srv/Empty "{}"
```