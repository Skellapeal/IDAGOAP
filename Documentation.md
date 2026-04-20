# Regressive IDA GOAP C++ API Reference for End Users
This document describes the public API of the core IDAGOAP C++ library, focusing on the types and methods exposed through the include/ headers and aggregated in rida_goap.h.
It is written for engine and gameplay programmers who want to integrate IDAGOAP into their game or simulation.

## Overview
IDAGOAP is a GOAP (Goal‑Oriented Action Planning) library that uses a regressive IDA* search over a symbolic world state to find action sequences that satisfy high‑level goals.
At a high level, you:

* Describe your world with the `world_state` key–value store.
* Define actions by subclassing `goap_action` and giving them preconditions and effects.
* Define goals as `motive` objects with desired world states and priorities.
* Plug those into a `goap_agent` which handles goal selection, planning, and execution for you.
* Optionally, use the lower‑level `rida_planner`, `async_planner`, and `plan_executor` APIs directly for custom control.

All symbols live in the `rida_goap` namespace.
## Top-Level Include
`rida_goal.h`

This is the main header you should include in user code.

```cpp
#include <rida_goal.h>
```
It aggregates the core public types:

* ``goap_types.h`` – core types (``state_value``, ``predicate_op``, ``state_condition``).
* ``world_state.h`` – world state container and helpers.
* ``goap_action.h`` – base action class.
* ``heuristic.h``, ``heuristics.h`` – base heuristic and built‑ins.
* ``plan_result.h`` – planning results.
* ``rida_planner.h`` – synchronous planner API.
* ``async_planner.h`` – asynchronous planner API.
* ``plan_executor.h`` – plan execution engine.
* ``motive.h`` – goal representation.
* ``goal_selector.h`` – goal selection and utility evaluation.

Use this single header unless you have a specific reason to include a sub‑header directly.

## Core Types and World Representation
``predicate_op``

##### Header: ``goap_types.h``.

Represents a comparison operator used in world state conditions and action preconditions.

```cpp
enum class predicate_op 
{
    Equal,
    NotEqual,
    Less,
    LessOrEqual,
    Greater,
    GreaterOrEqual
};
```
### Typical use‑cases:

* Define a precondition that health must be at least 50: ``predicate_op::GreaterOrEqual``.
* Define a precondition that a flag is not set: ``predicate_op::NotEqual``.

``state_value``

##### Header: ``goap_types.h``.

Variant type storing a world value:

```cpp
using state_value = std::variant<int, bool, float, std::string, std::vector<float>>;
```

Use‑cases:

Store numeric quantities (``int`` or ``float``), booleans, tags (``std::string``), or positions (``std::vector<float>`` for 2D/3D).

Used by ``world_state``, action effects, and stored conditions.

``state_condition``

##### Header: ``goap_types.h``.

Represents "world[key] predicate value".

```cpp
struct state_condition 
{
    state_value s_value;
    predicate_op predicate = predicate_op::Equal;

    state_condition();
    state_condition(state_value value, predicate_op comparison = predicate_op::Equal);

    bool evaluate(const world_state& world_model, const std::string& key) const;
};
```
Use‑cases:

* Built internally whenever you call ``goap_action::add_precondition``.
* You rarely construct this directly, but you can if you are building condition maps manually.

``world_state``

##### Header: ``world_state.h``.

Key–value map representing a snapshot of the world or a goal state.

```cpp
class world_state 
{
public:
    void set_state(const std::string& key, state_value value);
    std::optional<state_value> get_state(const std::string& key) const noexcept;

    bool has_state(const std::string& key) const noexcept;
    void remove_state(const std::string& key);

    const std::unordered_map<std::string, state_value>& get_states() const noexcept;

    void set_int(const std::string& key, int value);
    std::optional<int> get_int(const std::string& key) const;

    void set_bool(const std::string& key, bool value);
    std::optional<bool> get_bool(const std::string& key) const;

    void set_float(const std::string& key, float value);
    std::optional<float> get_float(const std::string& key) const;

    void set_string(const std::string& key, std::string value);
    std::optional<std::string> get_string(const std::string& key) const;

    void set_position(const std::string& key, float x, float y, float z = 0.0f);
    void set_position_2d(const std::string& key, float x, float y);
    std::optional<std::vector<float>> get_position(const std::string& key) const;

    void merge(const world_state& other);
    void merge_defaults(const world_state& other);

    bool satisfies(const std::unordered_map<std::string, state_condition>& goal_conditions) const;
    bool satisfies(const world_state& goal) const;

    bool operator==(const world_state& other) const;
};
```

Key behaviours and use‑cases:

* #####  Setting state:
    * ``set_state`` accepts a pre‑wrapped ``state_value``.
    * Typed helpers (``set_int``, ``set_bool``, etc.) are more convenient and safer.
* #####  Querying state:
  * get_state returns the raw state_value (wrapped in std::optional).
  * Typed getters (``get_int``, ``get_bool``, etc.) auto‑cast when the variant holds the desired type.
* #####  Positions:
  * ``set_position``and ``set_position_2d`` encode spatial data as a vector of floats under a key; used by the positional heuristics.
* #####  Merging:
    * ``merge(other)``: copy all keys from ``other``, overwriting any existing keys.
    * ``merge_defaults(other)``: copy only keys that are missing in this state.
* ##### Satisfaction:
  * ``satisfies(goal_conditions)`` checks if all conditions are met. 
  * ``satisfies(goal_state)`` compares against another world_state, used heavily in motives and goal checks.
  
Use ``world_state`` to represent:
* The agent’s current view of the world (through ``goap_agent::get_world_state``).
* Desired goal states for motive objects.
* Intermediate nodes in the planner’s search.

### Actions
``action_status``
#### Header: ``goap_action.h``.

```cpp
enum class action_status 
{
    Running,
    Succeeded,
    Failed
};
```
Return this from ``goap_action::on_tick`` to signal whether the action is still in progress, completed successfully, or failed.

``goap_action``
#### Header: ``goap_action.h``.

Abstract base class for all user‑defined actions.

```cpp
class goap_action 
{
protected:
    std::string name;
    int cost = 0;
    std::unordered_map<std::string, state_condition> preconditions;
    std::unordered_map<std::string, state_value>     effects;

public:
    using ptr = std::shared_ptr<goap_action>;
    using const_ptr = std::shared_ptrst goap_action>;

    goap_action(std::string name, int cost);
    virtual ~goap_action() = default;

    goap_action(const goap_action&) = default;
    goap_action(goap_action&&) noexcept = default;
    goap_action& operator=(const goap_action&) = default;
    goap_action& operator=(goap_action&&) noexcept = default;

    const std::string& get_name() const;
    int get_cost() const;
    const std::unordered_map<std::string, state_condition>& get_preconditions() const;
    const std::unordered_map<std::string, state_value>&     get_effects() const;

    void add_precondition(const std::string& key,
                          state_value value,
                          predicate_op comparison = predicate_op::Equal);

    void add_effect(const std::string& key, const state_value& value);

    virtual bool can_run() const;    // optional runtime guard
    bool check_preconditions(const world_state& world_model) const;
    void apply_effects(world_state& world_model) const;

    virtual bool on_start();
    virtual action_status on_tick(float delta_time) = 0;
    virtual void on_end(bool success);
    virtual void on_interrupt();
};
```

Key concepts and use‑cases:

* #### Construction:
  * ``goap_action("Attack", 2)`` sets a debug name and a planning cost.
* #### Preconditions:
    * ``add_precondition("HasWeapon", true)`` means the world must have ``HasWeapon == true`` for the action to be applicable.
    * Use non‑equal predicates to define range checks (e.g., ``Health >= 50``).
* #### Effects:
  * ``add_effect("EnemyDead", true)`` will be applied to the world when the action succeeds.
* #### Runtime hooks:
  * ``can_run()`` lets you reject actions dynamically (e.g., missing runtime resources) even if symbolic preconditions pass.
  * ``on_start()`` is called once per execution; return false to immediately fail the action.
  * ``on_tick(delta_time)`` is called each executor tick until you return ``Succeeded`` or ``Failed``.
  * ``on_end(success)`` is called when the action completes, for cleanup or side‑effects.
  * ``on_interrupt()`` is called if execution is interrupted mid‑action (e.g., plan aborted).

Use this as the base class for all behaviour units; the planner handles ordering, and the ``plan_executor``/``goap_agent`` call the hooks.

### Goals and Motives
``motive``
#### Header: ``motive.h``.

Represents a named goal with a desired world state and a priority.

```cpp
class motive
{
    std::string  name;
    world_state  goal_state;
    int          priority = 0;
    bool         persistent = false;

public:
    explicit motive(std::string name,
    world_state desired,
    int priority = 0);

    const std::string& get_name() const noexcept;
    int get_priority() const noexcept;
    const world_state& get_goal_state() const noexcept;

    bool is_persistent() const noexcept;
    void set_persistent(bool p) noexcept;

    bool is_satisfied(const world_state& world_model) const;

    void set_priority(int new_priority) noexcept;
};
```
Key behaviours and use‑cases:
* ``goal_state`` is a ``world_state`` describing when the motive is considered achieved, typically via ``goal_state.satisfies(world_model)``.
* priority drives default goal selection; higher priority motives are preferred when using the default utility evaluator.
* persistent motives cannot be dropped to a non‑positive priority; ``set_priority`` guarantees a minimum of 1 if persistent is true.

Typical usage:
* Construct motives from design‑time or run‑time data, then register them with a ``goap_agent``.
* Mark long‑term goals (like "Survive") as persistent and short‑term ones as non‑persistent.

``goal_selector``
#### Header: ``goal_selector.h``.

Utility class used by goap_agent to pick the best motive given the current world.

```cpp
class goal_selector 
{
public:
    using utility_evaluator = std::function<float(const motive&, const world_state&)>;

private:
    std::vector<std::shared_ptr<motive>> motives;
    utility_evaluator                     utility_fn;

public:
    goal_selector();

    void add_motive(std::shared_ptr<motive> motive);
    void remove_motive(const std::shared_ptr<motive>& motive);
    void clear_motives();

    std::shared_ptr<motive> find_motive(std::string_view name) const;

    void set_utility_evaluator(utility_evaluator evaluator);

    std::shared_ptr<motive> select_goal(const world_state& world_model) const;

    void set_motive_priority(std::string_view motive_name, int new_priority) const;
    void satisfy_motive(std::string_view motive_name) const;

    std::optional<world_state> select_goal_state(const world_state& world_model) const;

    std::vector<std::pair<std::shared_ptr<motive>, float>>
    evaluate_all_motives(const world_state& world_model) const;

    const std::vector<std::shared_ptr<motive>>& get_motives() const noexcept;
};
```
Key behaviours:
* **Default utility function:** returns ``motive.get_priority()``; you can override this to factor in distance, risk, or other world‑dependent criteria.
* #### Selection:
    * select_goal(world_model) evaluates all motives and returns the best candidate based on utility_fn (and their individual is_satisfied checks used by goap_agent).
* #### Utility introspection:
    * ``evaluate_all_motives(world_model)`` returns a vector of ``(motive, utility)`` pairs for UI or debugging.

End users typically interact with ``goal_selector`` through ``goap_agent`` rather than calling it directly.

### Heuristics
``heuristic`` (base)
#### Header: ``heuristic.h``.

Abstract base class for all heuristics.

```cpp
class heuristic 
{
public:
    virtual ~heuristic() = default;

    virtual int estimate(const world_state& world_model,
                         const world_state& goal) const = 0;
};
```
The planner uses ``estimate`` to guide search: lower values are better, and admissible heuristics preserve optimality.

#### Built‑in heuristics
#### Header: ``heuristics.h``.

Available implementations:

* ``zero_heuristic`` – always returns 0.
    * Use for correctness testing or when you have no domain knowledge (equivalent to Dijkstra’s algorithm).
* ``goal_count_heuristic`` – counts unsatisfied goal keys.
  * Standard GOAP heuristic: each mismatched or missing goal key contributes +1.
* ``euclidean_heuristic`` – Euclidean distance between world and goal positions for a given ``position_key``.
    * Ideal for navigation when positions are stored in ``world_state``.
* ``manhattan_heuristic`` – Manhattan distance between world and goal positions.
  * Good for grid‑based navigation without diagonals.
* ``composite_heuristic`` – weighted combination of multiple heuristics.
    * ``add_heuristic(h, weight)`` accumulates ``weight * h->estimate(...)`` for each sub‑heuristic.

Use‑cases:

* Choose ``goal_count_heuristic`` for general symbolic planning.
* Use ``euclidean_heuristic`` or ``manhattan_heuristic`` for path‑like problems.
* Combine several in ``composite_heuristic`` to mix symbolic and spatial costs.

### Planning APIs
``planner_options``
#### Header: ``rida_planner.h``.

Configuration for the regressive IDA* search.

```cpp
struct planner_options 
{
    int max_depth = std::numeric_limits<int>::max();
    int64_t max_nodes = std::numeric_limits<int64_t>::max();
    int time_budget_ms = -1; // -1 ignores time budget
    bool use_transposition_table= true;
    size_t max_transposition_size = std::numeric_limits<size_t>::max();
    std::stop_token cancel_token{};
};
```
Use‑cases:
* Limit search depth for debugging or specific gameplay constraints.
* Set a node or time budget to prevent expensive plans in worst‑case scenarios.
* Provide a ``cancel_token`` to abort long searches early (used by ``async_planner``).

``plan_status`` and ``plan_result``
#### Header: ``plan_result.h``.

```cpp
enum class plan_status 
{
    Success,
    NoSolutionExists,
    TimedOut,
    DepthLimitReached,
    NodeLimitReached,
    Cancelled
};

struct plan_result 
{
    std::vector<goap_action::const_ptr> actions;
    plan_status status = plan_status::Success;

    int64_t nodes_expanded = 0;
    int     final_cost     = 0;
    int     planning_time_ms = 0;

    bool success() const;
    bool is_trivially_satisfied() const; // Success and actions.empty()
    bool has_no_actions() const;
    size_t size() const;

    std::string status_string() const;
};
```
Use‑cases:
* Inspect actions to see the final sequence chosen by the planner.
* Use ``nodes_expanded``, ``final_cost``, and ``planning_time_ms`` for debugging or telemetry.
* ``status_string()`` returns a human‑readable reason when ``status != Success``.

``rida_planner``
#### Header: ``rida_planner.h``.

Low‑level synchronous planner. Useful if you want to run planning once without a ``goap_agent``.

```cpp
class rida_planner 
{
public:
    plan_result plan(const world_state& initial_state,
    const world_state& goal_state,
    std::span<goap_action::ptr> available_actions,
    const heuristic& heuristic,
    const planner_options& options);

    plan_result plan(const world_state& initial_state,
                     const world_state& goal_state,
                     std::span<goap_action::ptr> available_actions,
                     const heuristic& heuristic);
};
```
Use‑cases:
* Offline planning or batch simulations.
* Running a plan outside an agent loop, perhaps to pre‑compute routes.
* Testing heuristics or cost settings in isolation.

``async_planner``
#### Header: ``async_planner.h``.

Wrapper around ``rida_planner`` that runs ``plan`` in a background task.

```cpp
class async_planner 
{
public:
    using completion_callback = std::function<void(plan_result)>;

    async_planner();

    void plan_async(const world_state& initial_state,
                    const world_state& goal_state,
                    std::vector<goap_action::ptr> available_actions,
                    std::shared_ptr<heuristic> heuristic,
                    const planner_options& options = planner_options{});

    void plan_async(const world_state& initial_state,
                    const world_state& goal_state,
                    std::vector<goap_action::ptr> available_actions,
                    std::shared_ptr<heuristic> heuristic,
                    completion_callback callback,
                    const planner_options& options = planner_options{});

    bool is_planning_active() const noexcept;
    void cancel_planning();

    bool try_get_result(plan_result& result);
    plan_result wait_for_result();

    void set_completion_callback(completion_callback callback);
};
```
#### Key behaviours:
* ``plan_async`` launches work in a ``std::async`` task and sets ``is_planning`` while running.
* ``set_completion_callback`` registers a callback invoked when the plan is ready (unless cancelled).
* You can either rely on callbacks or poll ``try_get_result``/``wait_for_result``.
#### Use‑cases:
* Offload expensive planning to another thread in your game loop.
* Integrate with your own job system instead of ``goap_agent``.

### Plan Execution
``execution_status`` and ``execution_result``
#### Header: ``plan_executor.h``.

```cpp
enum class execution_status 
{
    Idle,
    Running,
    Success,
    Failed,
    Interrupted,
    NeedReplanning
};

struct execution_result 
{
    execution_status status = execution_status::Idle;
    size_t current_action_index = 0;
    std::string failure_reason;
};
```
Use‑cases:
* Report high‑level execution state to your AI system or debugging tools.
* Detect when a plan has failed or needs replanning to trigger higher‑level logic.

``plan_executor``
#### Header: ``plan_executor.h``.

Internal engine that carries out a ``plan_result`` by ticking actions.

```cpp
class plan_executor 
{
public:
    using replan_callback = std::function<plan_result(const world_state& current_world,
                                          const world_state& goal)>;

    explicit plan_executor(world_state* world_model = nullptr);

    void set_plan(plan_result plan, world_state goal);
    void set_world_model(world_state* world);
    void set_replan_callback(replan_callback callback);
    void set_auto_replan(bool enable);

    execution_status get_status() const noexcept;
    bool is_running() const noexcept;
    bool is_complete() const noexcept;

    size_t get_current_action_index() const noexcept;
    std::shared_ptrst goap_action> get_current_action() const noexcept;
    const plan_result& get_plan() const noexcept;

    execution_result tick(float delta_time);
    void interrupt();
    void reset();
};
```
Key behaviours and use‑cases:

* ``set_plan`` sets the current plan and goal state; the next call to tick will start the first action.
* ``set_world_model`` must point to the mutable ``world_state`` used by your actions.
* ``tick(delta_time)`` drives the lifecycle of each ``goap_action``:
  * Calls ``on_start`` when an action is first entered.
  * Calls ``on_tick`` each frame until it succeeds or fails. 
  * Calls ``on_end`` when done.
* ``set_replan_callback`` and ``set_auto_replan`` allow you to have the executor trigger replanning when an action fails due to invalidated preconditions.

Most users interact with this via ``goap_agent``, but it remains available for custom integration.

### Agent API (High‑Level)
``agent_status``
#### Header: ``goap_agent.h``.

```cpp
enum class agent_status 
{
    Idle,
    Planning,
    Executing,
    PlanFailed,
    Interrupted,
    NoMotives
};
```
Use‑cases:
* Introspect agent state for debugging, UI, or higher‑level AI logic.

``agent_config``
#### Header: ``goap_agent.h``.

```cpp
struct agent_config 
{
    planner_options options{};
    int  goal_recheck_interval = 5;
    bool replan_on_world_change = true;
    bool skip_replan_same_goal = true;
    int  max_consecutive_failures = 3;
    bool synchronous_planning = false;
};
```
Key fields:

* ``options`` – forwarded to ``rida_planner``/``async_planner`` for controlling search limits.
* ``goal_recheck_interval`` – how often (in ticks) to re‑evaluate the active goal when world changes.
* ``replan_on_world_change`` – whether to replan when the world model changes mid‑execution.
* ``skip_replan_same_goal`` – avoid unnecessary replans when the best goal hasn’t changed.
* ``max_consecutive_failures`` – how many failed planning attempts before going back to ``Idle`` instead of ``PlanFailed``.
* ``synchronous_planning`` – if ``true``, planning is done synchronously in ``kick_off_planning`` using a local ``rida_planner``; otherwise via ``async_planner``.

``goap_agent``
#### Header: ``goap_agent.h``.

High‑level façade that manages world state, motives, actions, planning, and execution.

```cpp
class goap_agent 
{
public:
    using status_changed_cb = std::function<void(agent_status prev, agent_status next)>;
    using goal_selected_cb = std::function<void(const motive&)>;
    using plan_found_cb = std::function<void(const plan_result&)>;
    using action_started_cb = std::function<void(const goap_action&)>;
    using action_finished_cb = std::function<void(const goap_action&, bool success)>;
    using goal_satisfied_cb = std::function<void(const motive&)>;

    explicit goap_agent(agent_config config = {});

    goap_agent(const goap_agent&) = delete;
    goap_agent& operator=(const goap_agent&) = delete;
    goap_agent(goap_agent&&) = delete;
    goap_agent& operator=(goap_agent&&) = delete;

    ~goap_agent() = default;

    void tick(float delta_time);

    // World access
    world_state& get_world_state();
    const world_state& get_world_state() const noexcept;

    void set_world_values(std::initializer_list<std::pair<std::string, state_value>> values);

    // Actions
    void add_action(goap_action::ptr action);
    void remove_action(const std::string& action_name);
    void clear_actions();
    const std::vector<goap_action::ptr>& get_actions() const noexcept;

    // Motives
    void add_motive(std::shared_ptr<motive> new_motive);
    void remove_motive(const std::shared_ptr<motive>& old_motive);
    void clear_motives();
    void satisfy_motive(std::string_view motive_name);
    void set_motive_priority(std::string_view motive_name, int new_priority) const;

    // Selection and heuristics
    void set_utility_evaluator(goal_selector::utility_evaluator fn);
    void set_heuristic(std::shared_ptr<heuristic> new_heuristic);

    // Control
    void interrupt();
    void resume();
    void force_replan();

    // Introspection
    agent_status get_status() const noexcept;
    std::shared_ptrst motive> get_active_motive() const noexcept;
    std::shared_ptrst goap_action> get_current_action() const noexcept;
    const plan_result& get_current_plan() const noexcept;
    bool is_planning() const noexcept;

    // Callbacks
    void set_on_status_changed (status_changed_cb cb);
    void set_on_goal_selected (goal_selected_cb cb);
    void set_on_plan_found (plan_found_cb cb);
    void set_on_action_started (action_started_cb cb);
    void set_on_action_finished (action_finished_cb cb);
    void set_on_goal_satisfied (goal_satisfied_cb cb);

    void mark_world_dirty();
};
```
Key behaviours and typical usage patterns:

* #### Construction:
  * ``goap_agent agent{};`` or with a custom ``agent_config`` to control planning behaviour.
* #### Ticking:
  * Call ``agent.tick(delta_time);`` once per frame.
  * Internally, the agent cycles through ``Idle → Planning → Executing`` states based on motives, actions, and world changes.
* #### World updates:
  * Use ``get_world_state()`` and ``world_state`` setters to reflect game state, then call ``mark_world_dirty()`` if you mutate the world directly.
  * ``set_world_values({{"Health", 100}, {"HasWeapon", true}});`` is a convenience method that sets multiple values and flags the world as dirty in one call.
* #### Actions:
  * Register all possible actions via ``add_action`` before or during runtime.
  * ``remove_action`` by name when you want to disable a behaviour.
* #### Motives:
  * Add motives with ``add_motive``, optionally adjusting priorities at runtime.
  * ``satisfy_motive(name)`` both marks the motive satisfied in the selector and interrupts any active planning or execution for that motive.
* #### Heuristic and selection:
  * ``set_heuristic`` must be called before planning; otherwise ``kick_off_planning`` will throw.
  * ``set_utility_evaluator`` lets you provide a custom utility function for goal selection.
* #### Control methods:
  * ``interrupt()`` cancels planning and execution and transitions to ``Interrupted``.
  * ``resume()`` moves back to ``Idle`` so the agent can start planning again.
  * ``force_replan()`` cancels the current plan and immediately triggers goal selection and planning.
* #### Callbacks:
  * Subscribe to hooks to drive animation, logging, or UI.
  * For example, ``set_on_action_started`` to log when a new action begins, ``set_on_goal_satisfied`` when a motive’s goal state becomes true.

With ``goap_agent``, you typically do not need to use ``rida_planner``, ``async_planner``, or ``plan_executor`` directly; the agent orchestrates them internally.