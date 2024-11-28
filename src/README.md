```bash
ros2 run call_service_in_lifecycle_node call_service_in_lifecycle_node
```
In another terminal
```bash
ros2 run demo_nodes_cpp add_two_ints_server
```
In another terminal
```bash
ros2 lifecycle set /simple_lifecycle configure
```