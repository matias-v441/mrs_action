# mrs_action

ROS 2 action servers for the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).

The MRS control stack takes flight commands as services (`control_manager/goto`,
`trajectory_generation/path`, `control_manager/reference`). A service call returns
as soon as the command is accepted, so the caller cannot tell when the flight
finishes. `mrs_action` wraps each of these services in a ROS 2 action. The action
calls the service, watches the tracker status, and finishes only after the UAV has
flown to the goal.

## Packages

| Package           | Contents                                                              |
|-------------------|-----------------------------------------------------------------------|
| `mrs_action_msgs` | Action definitions: `Goto`, `Path`, `ReferenceStamped`                 |
| `mrs_action`      | The action manager node (C++, plus a Python version), launch file, config, tests |

## Features

- **Three action types**, one per node instance, chosen by the `mode` parameter:

  | Mode        | Action                                | Wrapped service                                      |
  |-------------|---------------------------------------|------------------------------------------------------|
  | `goto`      | `mrs_action_msgs/action/Goto`             | `control_manager/goto` (`mrs_msgs/srv/Vec4`)         |
  | `path`      | `mrs_action_msgs/action/Path`             | `trajectory_generation/path` (`mrs_msgs/srv/PathSrv`) |
  | `reference` | `mrs_action_msgs/action/ReferenceStamped` | `control_manager/reference` (`mrs_msgs/srv/ReferenceStampedSrv`) |

- **Completion tracking.** The node subscribes to `control_manager/diagnostics` and
  uses `tracker_status.have_goal` to see when the flight starts and ends.
- **Waits for a busy UAV.** If the UAV is already flying to another goal when the
  action starts, the node waits until that flight ends before sending the request.
- **One goal at a time.** A new goal is rejected while another one is running.
- **Feedback** reports the current state of every goal (see below).
- **Failure reporting.** If the service rejects the request, the action is aborted
  and its result carries the service's message.

### States and feedback

Each goal goes through the states below. Every state change is published as
feedback in the `state` field.

| Value | State        | Meaning                                                        |
|-------|--------------|----------------------------------------------------------------|
| 0     | `IDLE`       | No goal is running                                             |
| 1     | `REQUESTING` | The service request has been sent; waiting for the flight to start |
| 2     | `WAITING`    | The UAV is busy with another goal; waiting for it to finish    |
| 3     | `FLYING`     | The UAV is flying to this goal                                 |

A typical goal goes `REQUESTING → FLYING`, and the action succeeds when the tracker
reports that it no longer has a goal. If the UAV was busy, the goal goes
`WAITING → REQUESTING → FLYING`.

### Result

| Field     | Type     | Description                                                   |
|-----------|----------|---------------------------------------------------------------|
| `success` | `bool`   | `true` if the flight finished                                 |
| `message` | `string` | `"finished flying"`, `"canceled"`, or the service's error message |

### Cancelling

A goal can be canceled only in the `IDLE` or `WAITING` state. After a request has
been sent to the UAV (`REQUESTING` or `FLYING`), cancel requests are rejected.

## Building

Requirements:

- ROS 2 with `rclcpp`, `rclcpp_action` and `rclpy`
- `mrs_msgs` from the MRS UAV System (ROS 2)
- A C++20 compiler

Clone the repository into the `src` folder of a colcon workspace, then build:

```bash
cd ~/ros2_ws
colcon build --packages-up-to mrs_action
source install/setup.bash
```

## Usage

### Launching

```bash
ros2 launch mrs_action action_manager.launch.py uav_name:=uav1
```

Launch arguments:

| Argument        | Default                                    | Description                                   |
|-----------------|--------------------------------------------|-----------------------------------------------|
| `uav_name`      | `$UAV_NAME`, or `uav1` if that is not set  | Namespace of the node                         |
| `custom_config` | `share/mrs_action/params/mrs_action_defaults.yaml` | Parameter file. A relative path is resolved against the current directory. |

The action server is at `/<uav_name>/action_manager/<mode>`, for example
`/uav1/action_manager/path`.

### Parameters

| Parameter         | Type   | Default | Description                                                    |
|-------------------|--------|---------|----------------------------------------------------------------|
| `mode`            | string | none (required) | `goto`, `path` or `reference`                       |
| `update_interval` | float  | `0.1` in the node, `1.0` in the default config | Seconds between state machine updates |

Example config:

```yaml
/**/action_manager:
  ros__parameters:
    mode: "goto"
    update_interval: 0.5
```

```bash
ros2 launch mrs_action action_manager.launch.py custom_config:=my_config.yaml
```

To serve more than one action type for the same UAV, launch one node per mode.

### Sending goals

**Goto** (`goal` is `[x, y, z, heading]`):

```bash
ros2 action send_goal --feedback /uav1/action_manager/goto \
  mrs_action_msgs/action/Goto "{goal: [10.0, 0.0, 3.0, 0.0]}"
```

**Path** (`path` is an `mrs_msgs/Path`):

```bash
ros2 action send_goal --feedback /uav1/action_manager/path \
  mrs_action_msgs/action/Path \
  "{path: {fly_now: true, points: [
     {position: {x: 5.0, y: 0.0, z: 3.0}, heading: 0.0},
     {position: {x: 5.0, y: 5.0, z: 3.0}, heading: 0.0}]}}"
```

Set `fly_now: true`. Otherwise the trajectory is loaded but not started, and the
action never gets past `REQUESTING`.

**Reference** (`header` + `mrs_msgs/Reference`):

```bash
ros2 action send_goal --feedback /uav1/action_manager/reference \
  mrs_action_msgs/action/ReferenceStamped \
  "{header: {frame_id: ''}, reference: {position: {x: 0.0, y: 0.0, z: 3.0}, heading: 0.0}}"
```

### Python implementation

`mrs_action/action_manager_node.py` is a Python version of the same node. The launch
file uses the C++ executable (`mrs_action_server`). To use the Python node, set
`executable="action_manager_node.py"` in
[action_manager.launch.py](mrs_action/launch/action_manager.launch.py). The Python
node ignores `update_interval` and always updates once per second.

## Tests

The tests are launch tests. Each one starts the action manager in `goto` mode
together with a mock control manager
([fake_goto_node.py](mrs_action/test/mock/fake_goto_node.py)), which serves
`control_manager/goto` and publishes fake diagnostics. They do not need a simulator
or the MRS UAV System running.

| Test                                               | Scenario                                                          |
|----------------------------------------------------|-------------------------------------------------------------------|
| [basic.py](mrs_action/test/basic.py)               | The UAV is busy at start; the goal waits, then flies and succeeds |
| [short_flight.py](mrs_action/test/short_flight.py) | The flight is shorter than `update_interval`, and is still detected |
| [delay_report.py](mrs_action/test/delay_report.py) | The tracker reports the flight 1 s after the service responds     |
| [multiple_goals.py](mrs_action/test/multiple_goals.py) | A second goal is rejected while the first runs; later goals run one after another |
| [fail.py](mrs_action/test/fail.py)                 | The service rejects the request; the action is aborted            |

Tests are built only when `ENABLE_TESTS` is on:

```bash
cd ~/ros2_ws
colcon build --packages-up-to mrs_action --cmake-args -DENABLE_TESTS=ON
source install/setup.bash

colcon test --packages-select mrs_action --event-handlers console_direct+
colcon test-result --verbose
```

Each test runs in an isolated ROS domain (`run_test_isolated.py`), so the tests do
not interfere with each other or with other ROS nodes on the network.

To run a single test, filter by name:

```bash
colcon test --packages-select mrs_action --ctest-args -R multiple_goals
```

The gtest unit tests in `test/unit/` are currently disabled in
[CMakeLists.txt](mrs_action/CMakeLists.txt).
