1. Run call_service_in_lifecycle_node
```bash
ros2 run call_service_in_lifecycle_node call_service_in_lifecycle_node
```
2. In another terminal
```bash
ros2 run demo_nodes_cpp add_two_ints_server
```
3. Call lifecycle configure action in another terminal
```bash
ros2 lifecycle set /simple_lifecycle configure
```
Or, call service in another terminal
```bash
ros2 service call /simple_lifecycle/add_two_ints std_srvs/srv/Empty "{}"
```