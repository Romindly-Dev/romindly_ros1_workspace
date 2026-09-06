# 03 Services and Custom Interfaces

> 中文版 / Chinese: [03_服务与自定义接口.md](../../10_ROS1基础/03_服务与自定义接口.md)

## Goals

- Understand the request-response model of services and how it fundamentally differs from topics
- Be able to write `.srv` files (using `AddTwoInts.srv` as an example) and master the `---`-separated structure
- Master the complete `message_generation` CMake configuration in `romindly_msgs`
- Read and understand the C++ / Python server and client code
- Be able to handle timeouts with `waitForExistence` / `wait_for_service`
- Be able to call services manually with `rosservice call`

Companion code: [service_demo](https://github.com/Romindly-Dev/romindly_ros1_tutorials/tree/main/service_demo), [romindly_msgs](https://github.com/Romindly-Dev/romindly_ros1_tutorials/tree/main/romindly_msgs)

## How It Works

A service is a **one-to-one, synchronous** remote procedure call (RPC): the client sends a request and **blocks waiting**; the server runs its callback, fills in the response, and returns it. A given service name can have only one server, but multiple clients.

```mermaid
sequenceDiagram
    participant C as client
    participant M as Master
    participant S as server
    S->>M: Register service /add_two_ints
    C->>M: Look up address of /add_two_ints
    M->>C: Return server address
    C->>S: Request {a: 3, b: 5}
    Note over S: Run callback sum = a + b
    S->>C: Response {sum: 8}
    Note over C: call() returns; it was blocking until now
```

Compared with topics: a topic is "broadcast, don't wait for a reply"; a service is "ask a specific party and wait for the answer before continuing". Therefore service callbacks must be **fast** — the client is stuck for the entire duration of the callback.

### srv File Structure

[romindly_msgs/srv/AddTwoInts.srv](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/romindly_msgs/srv/AddTwoInts.srv):

```
int64 a
int64 b
---
int64 sum
```

Above the `---` are the Request fields, below are the Response fields. Either section may be empty (e.g. a "trigger-style" service may have no request fields). Compilation generates three types: `AddTwoInts`, `AddTwoIntsRequest`, and `AddTwoIntsResponse`.

### message_generation Configuration Essentials

Whether custom interfaces build successfully depends entirely on these five pieces in [romindly_msgs/CMakeLists.txt](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/romindly_msgs/CMakeLists.txt) being complete:

```cmake
find_package(catkin REQUIRED COMPONENTS
  message_generation      # (1) pull in the code generator
  std_msgs                # dependency packages used in the messages
  actionlib_msgs          # needed for action generation
)

add_message_files(FILES RobotStatus.msg)    # (2) register msg files
add_service_files(FILES AddTwoInts.srv)     # (2') register srv files
add_action_files(FILES Countdown.action)    # (2'') register action files

generate_messages(                          # (3) trigger generation, declare inter-message dependencies
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(                             # (4) export run-time dependencies to downstream packages
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs
)
```

The companion [package.xml](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/romindly_msgs/package.xml) contains piece (5):

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

Common pitfalls:
- `generate_messages` must appear **before** `catkin_package`
- If a `.srv` references external types such as `std_msgs/Header`, `generate_messages(DEPENDENCIES ...)` must list the corresponding packages
- Downstream packages (such as `service_demo`) must depend on `romindly_msgs` in their own `find_package` and `package.xml`, and add `add_dependencies(target ${catkin_EXPORTED_TARGETS})` to each executable target, so that the interface code is generated before compilation

## Running the Example

We assume `cd ~/ws_romindly && catkin_make && source devel/setup.bash` has been done.

```bash
# Terminal 1
roscore
```

```bash
# Terminal 2: C++ server
rosrun service_demo add_two_ints_server
```

Expected output (Chinese log: "add_two_ints service is ready"):

```
[ INFO] [1757130000.100]: add_two_ints 服务已就绪
```

```bash
# Terminal 3: Python client (cross-language call)
rosrun service_demo add_two_ints_client.py 3 5
```

Expected output (one line from the client, one from the server):

```
[INFO] [1757130002.345]: sum = 8
[ INFO] [1757130002.344]: request: a=3 b=5 -> sum=8
```

The C++ client works the same way: `rosrun service_demo add_two_ints_client 3 5`.

### Command-Line Tools

```bash
rosservice list                        # list all services
rosservice info /add_two_ints          # show type and server node
rossrv show romindly_msgs/AddTwoInts   # show the srv definition
rosservice call /add_two_ints "a: 7
b: 35"
```

Expected output of `rosservice call`:

```
sum: 42
```

Arguments are in YAML format; for simple types you can also write `rosservice call /add_two_ints 7 35`. Pressing Tab twice auto-completes the argument template.

## Code Walkthrough

### C++ Server [service_demo/src/add_two_ints_server.cpp](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/service_demo/src/add_two_ints_server.cpp)

```cpp
bool add(romindly_msgs::AddTwoInts::Request& req,
         romindly_msgs::AddTwoInts::Response& res)
{
  res.sum = req.a + req.b;
  return true;   // true = call succeeded; false makes the client's call() return failure
}

ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
ros::spin();
```

- The callback signature is fixed as `(Request&, Response&) -> bool`; filling in `res` fills in the response
- Returning `false` means "the service failed to handle the request", and the client's `client.call()` returns false
- `ros::spin()` dispatches incoming requests, just as with topic subscriptions

### C++ Client [service_demo/src/add_two_ints_client.cpp](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/service_demo/src/add_two_ints_client.cpp)

```cpp
ros::ServiceClient client =
    nh.serviceClient<romindly_msgs::AddTwoInts>("add_two_ints");

romindly_msgs::AddTwoInts srv;
srv.request.a = atoll(argv[1]);
srv.request.b = atoll(argv[2]);

// Block waiting for the service to come online, at most 5 seconds
if (!client.waitForExistence(ros::Duration(5.0)))
{
  ROS_ERROR("等待 add_two_ints 服务超时，请先启动服务端");  // "Timed out waiting for add_two_ints; start the server first"
  return 1;
}

if (client.call(srv))
  ROS_INFO("sum = %ld", (long)srv.response.sum);
```

- The `srv` object contains both the `request` and `response` halves
- **`waitForExistence(timeout)`**: the client may start before the server, and a direct `call` would fail immediately. This waits with a timeout for the service to register; on timeout it returns false, letting you report a clear error instead of hanging forever. Pass `ros::Duration(-1)` to wait indefinitely
- `call()` is a synchronous blocking call; only a return value of true means the response is valid

### Python Version [add_two_ints_server.py](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/service_demo/scripts/add_two_ints_server.py) / [add_two_ints_client.py](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/service_demo/scripts/add_two_ints_client.py)

Server:

```python
def handle_add(req):
    return AddTwoIntsResponse(sum=req.a + req.b)

rospy.Service("add_two_ints", AddTwoInts, handle_add)
rospy.spin()
```

The Python callback receives only `req`; **the response is given via the return value** (return an `AddTwoIntsResponse` object; raising an exception signals call failure).

Timeout handling on the client:

```python
try:
    rospy.wait_for_service("add_two_ints", timeout=5.0)
except rospy.ROSException:
    rospy.logerr("等待 add_two_ints 服务超时，请先启动服务端")  # "Timed out waiting for add_two_ints; start the server first"
    sys.exit(1)

add_two_ints = rospy.ServiceProxy("add_two_ints", AddTwoInts)
resp = add_two_ints(a, b)
```

- `rospy.wait_for_service` corresponds to C++'s `waitForExistence`, but on timeout it **raises `rospy.ROSException`** rather than returning false, so it must be caught with try/except
- `ServiceProxy` produces a callable object; just pass arguments in the order of the request fields. If the server crashes during the call, `rospy.ServiceException` is raised — robust code should catch it as well

## Hands-on Exercises

1. Add `MultiplyTwoInts.srv` to `romindly_msgs` (request `int64 a` / `int64 b`, response `int64 product`), register it in `add_service_files` in CMakeLists, rebuild, verify with `rossrv show`, then write a multiplication server modeled after [add_two_ints_server.py](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/service_demo/scripts/add_two_ints_server.py) and test it with `rosservice call`.
2. Modify [add_two_ints_server.cpp](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/service_demo/src/add_two_ints_server.cpp): return `false` when `a` or `b` is negative, and observe how the client's `client.call(srv)` behavior changes and what error is printed.

## FAQ

**Q1: The client reports `service [/add_two_ints] unavailable` or hangs forever?**
The server is not running or the service names differ. Confirm the service exists with `rosservice list`; mind the namespaces. The code in this suite uses a 5-second timeout throughout, so a timeout error is your cue to start the server first.

**Q2: After changing the .srv, Python fails with `ImportError: cannot import name ...`?**
The interface code was not regenerated. Re-run `catkin_make` and `source devel/setup.bash`, and confirm the new file is registered in `add_service_files`.

**Q3: Can a service callback do time-consuming work (e.g. sleep 30 seconds)?**
Not recommended. The client blocks the entire time the callback runs, and with single-threaded spin in C++, other callbacks in the same node are stalled too. Long tasks should use an Action instead (see the next part).

**Q4: `rosservice call` reports a YAML parse error?**
With multiple fields, mind the newline-inside-quotes syntax: `rosservice call /add_two_ints "a: 7` (press Enter) `b: 35"`, or use the inline form `"{a: 7, b: 35}"`.

**Q5: Build fails with `Could not find messages which '.../AddTwoInts.srv' depends on`?**
The `.srv` references an undeclared message package. Add the package in all three places: `find_package(... COMPONENTS)`, `generate_messages(DEPENDENCIES ...)`, and `package.xml`.
