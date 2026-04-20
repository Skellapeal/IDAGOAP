# IDAGOAP

A platform-agnostic, header-clean **Goal-Oriented Action Planning (GOAP)** library written in C++20, built as the core engine for an Unreal Engine 5 plugin. IDAGOAP provides a complete AI planning pipeline — from world state modelling and action definition through to async planning, plan execution, and a fully observable agent — in a single static library with no external runtime dependencies.

> **Status:** Active development — Final Year Project (2025–2026).
> UE5 plugin integration is in progress as a separate layer on top of this core library.

---

## Table of Contents

- [Features](#features)
- [Architecture Overview](#architecture-overview)
- [Requirements](#requirements)
- [Building](#building)
- [Quick Start](#quick-start)
- [Core Concepts](#core-concepts)
  - [World State](#world-state)
  - [Actions](#actions)
  - [Motives](#motives)
  - [The Agent](#the-agent)
  - [Agent Status](#agent-status)
  - [Agent Config](#agent-config)
- [Advanced Usage](#advanced-usage)
  - [Custom Heuristics](#custom-heuristics)
  - [Async Planning](#async-planning)
  - [Planning Without an Agent](#planning-without-an-agent)
  - [Callbacks](#callbacks)
- [Project Structure](#project-structure)
- [Running Tests](#running-tests)
- [CMake Integration](#cmake-integration)

---

## Features

- **A\* GOAP planner** with a transposition table to avoid revisiting duplicate world states
- **Synchronous and asynchronous planning** — async planning runs on a background thread with cancellation support
- **Observable agent state machine** — `Idle`, `Planning`, `Executing`, `PlanFailed`, `Interrupted`, `NoMotives`
- **Priority-based goal selection** with a pluggable utility evaluator for custom goal scoring
- **Persistent and one-shot motives** — goals can survive satisfaction or be consumed once achieved
- **Plan executor** with per-action lifecycle hooks (`on_start`, `on_tick`, `on_end`, `on_interrupt`)
- **Configurable replanning** — react to world state changes, skip redundant replans for the same goal, and cap consecutive failures
- **Stock heuristics** included: `goal_count_heuristic`, `weighted_goal_count_heuristic`, `null_heuristic`
- **C++20**, no external runtime dependencies, CMake-native with `GNUInstallDirs` install support
- Auto-copies headers and the built `.lib` into a UE5 plugin directory on build (configurable via CMake cache)

---

## Architecture Overview

<p align="center">
  <img width="607" height="333" alt="IDAGOAP Architecture" src="https://github.com/user-attachments/assets/18126b66-eb76-4ff2-bf58-4f78055115fc" />
</p>

The agent owns the entire pipeline. On each `tick()` it selects the highest-utility unsatisfied motive, fires off planning (sync or async), and steps the executor through the resulting action sequence — automatically replanning when the world changes or an action fails.

---

## Requirements

| Requirement | Minimum |
|---|---|
| C++ Standard | C++20 |
| CMake | 3.22 |
| Compiler | MSVC 19.29+, GCC 11+, Clang 13+ |
| Test framework | GoogleTest (fetched automatically via CMake) |

---

## Building

```bash
git clone https://github.com/Skellapeal/IDAGOAP.git
cd IDAGOAP
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

To **skip tests**:

```bash
cmake -B build -DIDAGOAP_BUILD_TESTS=OFF
cmake --build build
```

To **point the UE5 plugin copy** at a different directory:

```bash
cmake -B build \
  -DUE_PLUGIN_INCLUDE_DIR="C:/MyProject/Plugins/IDAGOAP/ThirdParty/IDAGOAPCore/Include" \
  -DUE_PLUGIN_LIB_DIR="C:/MyProject/Plugins/IDAGOAP/ThirdParty/IDAGOAPCore/Lib/Win64"
```

After a successful build, `IDAGOAP.lib` and all public headers are automatically copied to the configured UE plugin directories.

---

## Quick Start

Include the single aggregator header:

```cpp
#include "rida_goap.h"
using namespace rida_goap;
```

### 1. Define an action

Subclass `goap_action` and override `on_tick`:

```cpp
class pick_up_weapon : public goap_action
{
public:
    pick_up_weapon() : goap_action("PickUpWeapon", /*cost=*/1) 
    {
        add_precondition("has_weapon", state_value{false});
        add_effect("has_weapon",       state_value{true});
    }

    action_status on_tick(float /*delta_time*/) override
    {
        // Perform the actual work here (animate, move, etc.)
        return action_status::Succeeded;
    }
};
```

### 2. Set up an agent

```cpp
// Configure the agent
agent_config cfg;
cfg.synchronous_planning   = true;   // plan on the main thread (good for testing)
cfg.replan_on_world_change = true;
cfg.goal_recheck_interval  = 5;      // re-evaluate goals every 5 ticks

goap_agent agent(cfg);

// Heuristic (required before planning)
agent.set_heuristic(std::make_shared<goal_count_heuristic>());

// World state
agent.set_world_values({
    {"has_weapon", state_value{false}},
    {"enemy_dead", state_value{false}}
});

// Actions
agent.add_action(std::make_shared<pick_up_weapon>());

auto shoot = std::make_shared<goap_action_impl>(/* ... */);
// ... configure shoot ...
agent.add_action(shoot);

// Goal (motive)
world_state goal;
goal.set_bool("enemy_dead", true);
agent.add_motive(std::make_shared<motive>("KillEnemy", goal, /*priority=*/10));
```

### 3. Tick every frame

```cpp
// In your game loop:
void update(float delta_time)
{
    agent.tick(delta_time);
}
```

That's it. The agent selects its goal, plans a sequence of actions, and executes them automatically.

---

## Core Concepts

### World State

`world_state` is the shared model of reality that both the planner and executor read from. It stores named key–value pairs where values can be `bool`, `int`, `float`, `std::string`, or a 3D position.

```cpp
world_state ws;
ws.set_bool  ("is_armed",    true);
ws.set_int   ("ammo",        12);
ws.set_float ("health",      0.75f);
ws.set_string("target_name", "Player");
ws.set_position("patrol_pos", 100.f, 0.f, 200.f);

// Read back
bool armed  = ws.get_bool("is_armed").value_or(false);
int  ammo   = ws.get_int ("ammo").value_or(0);

// Check goal satisfaction
world_state goal;
goal.set_bool("is_armed", true);
bool satisfied = ws.satisfies(goal); // true
```

Mutate the agent's world through `goap_agent::set_world_values()` or directly via `goap_agent::get_world_state()` — always call `mark_world_dirty()` after direct mutation so the agent reacts to the change.

```cpp
// Preferred — automatically marks dirty
agent.set_world_values({{"ammo", state_value{0}}});

// Direct mutation — must mark dirty manually
agent.get_world_state().set_int("ammo", 0);
agent.mark_world_dirty();
```

---

### Actions

Actions are the atomic steps the planner chains together to reach a goal. Subclass `goap_action` and set preconditions and effects in the constructor:

```cpp
class reload_action : public goap_action
{
public:
    reload_action() : goap_action("Reload", /*cost=*/2)
    {
        add_precondition("ammo",         state_value{0});
        add_precondition("has_magazine", state_value{true});
        add_effect("ammo",               state_value{30});
        add_effect("has_magazine",       state_value{false});
    }

    bool on_start() override
    {
        // Called once when this action begins executing.
        // Return false to abort immediately.
        return true;
    }

    action_status on_tick(float delta_time) override
    {
        // Called every frame while this action is active.
        // Return Succeeded, Failed, or Running.
        timer_ -= delta_time;
        return timer_ <= 0.f ? action_status::Succeeded : action_status::Running;
    }

    void on_end(bool success) override
    {
        // Called once when the action finishes (success or failure).
    }

    void on_interrupt() override
    {
        // Called if the plan is cancelled mid-action.
    }

private:
    float timer_ = 1.5f;
};
```

**Precondition comparisons** — by default preconditions use `Equal`, but you can use other operators:

```cpp
// Requires ammo > 0
add_precondition("ammo", state_value{0}, predicate_op::Greater);

// Requires health <= 25
add_precondition("health", state_value{25}, predicate_op::LessEqual);
```

Available operators: `Equal`, `NotEqual`, `Less`, `LessEqual`, `Greater`, `GreaterEqual`.

**Action cost** drives the planner to prefer cheaper paths. Higher cost = less preferred.

---

### Motives

A `motive` is a named goal: a desired `world_state` with a priority. The agent's `goal_selector` picks the highest-utility unsatisfied motive each planning cycle.

```cpp
// One-shot motive — satisfied and removed once goal is met
world_state kill_goal;
kill_goal.set_bool("enemy_dead", true);
auto kill = std::make_shared<motive>("KillEnemy", kill_goal, /*priority=*/10);
agent.add_motive(kill);

// Persistent motive — re-evaluated even after satisfaction
world_state patrol_goal;
patrol_goal.set_bool("at_patrol_point", true);
auto patrol = std::make_shared<motive>("Patrol", patrol_goal, /*priority=*/1);
patrol->set_persistent(true);
agent.add_motive(patrol);
```

Change priority at runtime to dynamically shift agent behaviour:

```cpp
agent.set_motive_priority("KillEnemy", 99); // urgent
agent.set_motive_priority("Patrol",     1); // background
```

Manually satisfy a motive to force the agent to abandon it and re-select:

```cpp
agent.satisfy_motive("KillEnemy");
```

---

### The Agent

`goap_agent` is the primary entry point. It owns a world state, a set of actions, a goal selector, and the planning/execution pipeline. It is **non-copyable and non-movable** — create it on the heap or as a long-lived member.

| Method | Description |
|---|---|
| `tick(float dt)` | Drive the agent forward one frame. Call once per game loop iteration. |
| `interrupt()` | Immediately halt planning and execution. Transitions to `Interrupted`. |
| `resume()` | Resume from `Interrupted` back to `Idle`. |
| `force_replan()` | Cancel the current plan and start a fresh planning cycle. |
| `mark_world_dirty()` | Signal that the world state has been mutated externally. |

---

### Agent Status

The agent exposes a fine-grained status via `get_status()` that maps directly to its internal phase:

| Status | Meaning |
|---|---|
| `Idle` | Between goals. Will select a new goal on the next tick. |
| `NoMotives` | No motives are registered or all have priority ≤ 0. Agent will not self-start. |
| `Planning` | Planner is running (background thread or synchronously). |
| `Executing` | A plan has been found and actions are being stepped. |
| `PlanFailed` | The planner returned no valid plan. Will retry up to `max_consecutive_failures`. |
| `Interrupted` | Manually halted. Will not resume until `resume()` is called. |

---

### Agent Config

All behaviour tuning lives in `agent_config`, passed to the constructor:

```cpp
agent_config cfg;
cfg.goal_recheck_interval    = 5;     // Re-evaluate goal selection every N ticks (0 = every tick)
cfg.replan_on_world_change   = true;  // Interrupt and replan when world state changes
cfg.skip_replan_same_goal    = true;  // Don't replan if the selected goal hasn't changed
cfg.max_consecutive_failures = 3;     // Transitions to Idle after N consecutive plan failures
cfg.synchronous_planning     = false; // true = plan on calling thread (blocks); false = background thread
```

`cfg.options` exposes the underlying `planner_options` for node/depth limits and timeouts:

```cpp
cfg.options.max_nodes  = 10000;
cfg.options.max_depth  = 32;
cfg.options.timeout_ms = 50;
```

---

## Advanced Usage

### Custom Heuristics

The stock heuristics cover most cases:

```cpp
agent.set_heuristic(std::make_shared<goal_count_heuristic>());          // counts unsatisfied goal conditions
agent.set_heuristic(std::make_shared<weighted_goal_count_heuristic>(2.0f)); // weighted variant
agent.set_heuristic(std::make_shared<null_heuristic>());                // always returns 0 (pure Dijkstra)
```

For a custom heuristic, subclass `heuristic`:

```cpp
class distance_heuristic : public heuristic
{
public:
    float calculate(const world_state& current,
                    const world_state& goal) const override
    {
        // Return an admissible estimate of remaining cost.
        // Lower = closer to goal.
        float cost = 0.f;
        for (const auto& [key, _] : goal.get_states())
            if (!current.has_state(key)) cost += 1.f;
        return cost;
    }
};
```

### Custom Goal Selection (Utility Evaluator)

Override the default priority-based goal selection with any custom scoring function:

```cpp
agent.set_utility_evaluator([](const motive& m, const world_state& world) -> float
{
    // Example: urgency scales with missing health
    if (m.get_name() == "Heal")
    {
        float hp = world.get_float("health").value_or(1.f);
        return (1.f - hp) * 100.f; // more urgent the lower the health
    }
    return static_cast<float>(m.get_priority());
});
```

---

### Async Planning

By default (`synchronous_planning = false`) planning runs on a background thread. The agent handles synchronisation automatically — use `tick()` as normal.

If you want to manage an `async_planner` directly:

```cpp
async_planner planner;

planner.plan_async(
    world, goal, actions, heuristic,
    [](const plan_result& result)
    {
        if (result.success())
        {
            // result.actions contains the ordered action sequence
        }
    });

// Later, on a different thread or after a delay:
if (!planner.is_planning_active())
{
    plan_result result;
    if (planner.try_get_result(result))
    {
        // consume result
    }
}

// Cancel at any time:
planner.cancel_planning();
```

---

### Planning Without an Agent

Use `rida_planner` directly for one-shot planning without any agent lifecycle:

```cpp
rida_planner planner;
goal_count_heuristic h;

const plan_result result = planner.plan(
    initial_world,
    goal_world,
    std::span{actions},
    h,
    planner_options{});

if (result.success())
{
    for (const auto& action : result.actions)
        std::cout << action->get_name() << "\n";

    std::cout << "Total cost: " << result.final_cost << "\n";
    std::cout << "Nodes expanded: " << result.nodes_expanded << "\n";
}
else
{
    std::cout << "Planning failed: " << result.status_string() << "\n";
}
```

---

### Callbacks

Hook into any agent event with callbacks — useful for logging, animation triggers, or UI updates:

```cpp
agent.set_on_status_changed([](agent_status prev, agent_status next)
{
    // Fires on every status transition
});

agent.set_on_goal_selected([](const motive& m)
{
    // Fires when a new goal is chosen
    std::cout << "Goal selected: " << m.get_name() << "\n";
});

agent.set_on_plan_found([](const plan_result& r)
{
    // Fires when a valid plan is produced
    std::cout << "Plan found with " << r.actions.size() << " actions\n";
});

agent.set_on_action_started([](const goap_action& a)
{
    // Fires when an action begins executing
});

agent.set_on_action_finished([](const goap_action& a, bool success)
{
    // Fires when an action completes or fails
});

agent.set_on_goal_satisfied([](const motive& m)
{
    // Fires when a goal's conditions are met in the world
});
```

---

## Project Structure

```text
IDAGOAP/
├── include/
│ ├── rida_goap.h ← Single include for end-users
│ ├── goap_agent.h ← Primary agent interface
│ ├── goap_action.h ← Base class for all actions
│ ├── world_state.h ← World model / key-value state store
│ ├── motive.h ← Named goal with priority
│ ├── goal_selector.h ← Priority/utility-based goal selection
│ ├── rida_planner.h ← Synchronous A* GOAP planner
│ ├── async_planner.h ← Thread-backed async planning wrapper
│ ├── plan_executor.h ← Steps through a plan, calls action hooks
│ ├── plan_result.h ← Planner output and status codes
│ ├── heuristic.h ← Abstract heuristic base
│ ├── heuristics.h ← Stock heuristic implementations
│ ├── goap_types.h ← state_value, state_condition, predicate_op
│ └── transposition_table.h← Visited-state cache for the planner
├── src/ ← Implementations (not for direct inclusion)
├── tests/ ← GoogleTest suite
└── CMakeLists.txt
```
---

## Running Tests

```bash
cmake -B build -DIDAGOAP_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Or run the test binary directly:

```bash
./build/tests/idagoap_tests
```

---

## CMake Integration

After installing, consume the library via its exported CMake target:

```cmake
find_package(rida_goap REQUIRED)

target_link_libraries(my_target PRIVATE rida_goap::rida_goap)
```

Or, if consuming as a subdirectory:

```cmake
add_subdirectory(IDAGOAP)
target_link_libraries(my_target PRIVATE rida_goap::rida_goap)
```

The alias target `rida_goap::rida_goap` is defined in `CMakeLists.txt` and is the recommended way to link in both cases.
