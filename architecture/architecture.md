# AcadBot Courier — Architecture

Status: **generic / early draft.** Picked up again next session. Nothing here
is final except the two items explicitly marked DECIDED. Everything else is
either OPEN or a leaning.

See `assignment.md` in this folder for the requirements every decision here
must trace back to. See `claude.md` for how Maya and Claude work together on
this project.

---

## 1. Packages

**DECIDED.** Two new packages, alongside the existing `acadbot_control`,
`acadbot_navigation`, etc:

- `acadbot_courier_msgs` — interfaces only. No C++ logic, no node code.
  Holds the `.srv`, `.action`, and any reusable `.msg` (e.g. a `NamedLocation`
  or similar pose-with-a-name message, reused across the service and action
  definitions instead of duplicating fields).
- `acadbot_courier` — the nodes and their config/launch files.

Rationale: mirrors the existing `nav2_msgs` / `nav2_bt_navigator` split, keeps
`acadbot_courier` buildable against interfaces without dragging node-level
build deps into anything that just wants to speak the interface (e.g. future
BT leaf nodes, or a test harness).

## 2. Node topology

**DECIDED (topology), OPEN (internals).** Distributed, not one cramped node.
At minimum:

- **dispatcher** node — hosts the delivery-request *service* (requirement 1/2).
  Validates the request (known locations? currently free?), and if accepted,
  hands the job off to the executor and relays progress/result back up to
  whatever is watching the *action* the requester used to track the job.
- **executor** node — pure Nav2 `navigate_to_pose` action client
  (requirement 5). Knows nothing about job IDs, YAML location names, or
  rejection reasons — it receives a leg (a pose) and reports back
  progress/success/failure/canceled for that leg. Doesn't know it's part of
  a "delivery"; that concept lives in the dispatcher.

This is the `see/sense/think/act` + `orchestrator` shape carried over from the
`smart_fire_extinguisher` FYP: dispatcher is the orchestrator, executor is the
"act" layer, Nav2 is the actuator underneath that.

**Open question — dispatcher ↔ executor link.** Leaning: dispatcher is an
action client to the executor's action server, so the same
goal/feedback/result shape flows all the way down
(requester → dispatcher → executor → Nav2). Not yet locked in — revisit once
the two `.action` definitions (requester-facing and internal) are drafted,
because if they end up nearly identical we should ask whether the executor
even needs to be a separate action server or whether a simpler synchronous
call (still not a topic — see reasoning log) does the job.

**Rejected: topic for dispatcher → executor handoff.** A topic is
fire-and-forget with no delivery/ack guarantee — wrong fit for "hand off
exactly one job and know whether it was taken." The original worry (shared
queue between two nodes causing memory races) doesn't actually apply here
regardless: separate nodes are separate processes, there's no shared memory
to race on unless one is deliberately set up. A topic, service, and action are
all race-free between processes; they differ in delivery guarantee and
reply shape, not in thread-safety. Rosbag was floated as unrelated but
good — operational, not architectural, wire in near the end via
`ros2 bag record` on the feedback/status topics.

## 3. Concurrency / job admission

**OPEN — contradicts itself in the current notes, needs one more pass.**
Two behaviors were both described and they are opposites:

- "Queue tasks" → job B waits, runs automatically after job A finishes.
- "If it can't do it right now it's dropped" (the 'smart mesh' framing) →
  reject-if-busy, job B never runs unless re-requested.

Physical constraint either way: one robot, one Nav2 client, only one job
can ever be *executing* at a time — this is not about true concurrency, only
about what the dispatcher does with an incoming request while another job is
active. Resolve next session before touching the dispatcher's state machine;
the two `.srv`/`.action` result fields (and the rejection-reason wording)
depend on which one we pick.

## 4. Failure / retry semantics

**Leaning, not fully locked.** A job is two legs: pickup, then dropoff.

- Retry count is YAML-configured (requirement 8), applies per leg.
- If the pickup leg exhausts retries, the dropoff leg never starts — job ends
  FAILED, reason names the pickup location. No pretending a pickup happened.
- If the dropoff leg exhausts retries after a successful pickup, job ends
  FAILED, reason names the dropoff location. Open real-world gap, worth a
  slide: robot is now sitting somewhere holding a "package" it can't
  deliver — out of scope to solve, but should be acknowledged, not hidden.
- Maya raised: should there be an explicit alarm/stop state distinct from a
  normal FAILED result, given a physical package can't just be "dropped
  anywhere"? Not resolved — revisit alongside decision 3, since both are
  about what happens after the happy path breaks.

## 5. Location configuration (YAML)

**Mechanism decided, format TBD.** ROS 2 parameters cannot be a custom
message type — only primitives and homogeneous arrays of primitives. The
name→pose map (`reception`, `lab_bench`, ...) will use nested parameter
namespacing, e.g.:

```yaml
locations:
  reception: {x: 1.0, y: 2.0, yaw: 0.0}
  lab_bench: {x: 4.5, y: -1.2, yaw: 1.57}
```

read back via `list_parameters`/dotted param names (`locations.reception.x`,
etc.) — this differs from `patrol_commander`'s flat `[x,y,yaw,...]` list
because we need name-keyed lookup, not an ordered sequence. Separately, and
not to be confused with this: a reusable `NamedLocation`-style **message**
(composition, e.g. reused across the `.srv` and `.action` field
definitions) is a legitimate, unrelated ROS 2 mechanism — that's about
interface design, not parameter loading.

## 6. Bonus — Behavior Tree

**Sequenced deliberately last.** Build requirements 1–9 first as a plain
state machine (dispatcher/executor callback chaining, same shape as
`patrol_commander` but two legs + retry instead of N waypoints), get it fully
working end to end, *then* port the mission logic into BehaviorTree.CPP.
Reasoning: designing leaf-node boundaries around a BT before the working
version exists means designing blind, and BT.CPP has its own footguns
(blackboard types, tick semantics, async node halting on cancel) that
shouldn't be debugged simultaneously with the Nav2 action-client logic.

## 7. Not yet discussed

- Exact `.srv` and `.action` field lists (goal/feedback/result contents).
- Timeout handling (requirement 8 mentions timeouts as a configurable — not
  yet designed).
- Launch file structure for requirement 9 (single command bring-up including
  the new nodes).
- Package `CMakeLists.txt`/`package.xml` details for the two new packages.
