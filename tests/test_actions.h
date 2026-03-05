//
// Created by Niall Ó Colmáin on 04/03/2026.
//

#ifndef IDAGOAP_TEST_ACTIONS_H
#define IDAGOAP_TEST_ACTIONS_H

#include "../gaction.h"
#include "../gtypes.h"

/**
 * Test action: Gather wood from the environment
 * Preconditions: none
 * Effects: has_wood = 1
 */
class gather_wood_action : public gaction
{
public:
    gather_wood_action() : gaction("GatherWood", 10)
    {
        effects["has_wood"] = gvalue{1};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

/**
 * Test action: Craft a tool using wood
 * Preconditions: has_wood = 1
 * Effects: has_tool = 1, has_wood = 0 (consumes wood)
 */
class craft_tool_action : public gaction
{
public:
    craft_tool_action() : gaction("CraftTool", 5)
    {
        preconditions["has_wood"] = gvalue{1};

        effects["has_tool"] = gvalue{1};
        effects["has_wood"] = gvalue{0};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

/**
 * Test action: Gather stone from the environment
 * Preconditions: none
 * Effects: has_stone = 1
 */
class gather_stone_action : public gaction
{
public:
    gather_stone_action() : gaction("GatherStone", 15)
    {
        effects["has_stone"] = gvalue{1};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

/**
 * Test action: Build shelter using wood and stone
 * Preconditions: has_wood = 1, has_stone = 1
 * Effects: has_shelter = 1, has_wood = 0, has_stone = 0
 */
class build_shelter_action : public gaction
{
public:
    build_shelter_action() : gaction("BuildShelter", 20)
    {
        preconditions["has_wood"] = gvalue{1};
        preconditions["has_stone"] = gvalue{1};

        effects["has_shelter"] = gvalue{1};
        effects["has_wood"] = gvalue{0};
        effects["has_stone"] = gvalue{0};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

/**
 * Test action: An action that's conditionally disabled
 * Used to test can_run() filtering
 */
class disabled_action : public gaction
{
public:
    disabled_action() : gaction("DisabledAction", 1)
    {
        effects["should_not_use"] = gvalue{1};
    }

    bool can_run() const override { return false; }
    bool setup() override { return true; }
    bool end() override { return true; }
};

class scout_action : public gaction
{
public:
    scout_action() : gaction("Scout", 8)
    {
        effects["enemy_visible"] = gvalue{true};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

class hide_action : public gaction
{
public:
    hide_action() : gaction("Hide", 3)
    {
        preconditions["enemy_visible"] = gvalue{true};

        effects["is_hidden"] = gvalue{true};
        effects["enemy_visible"] = gvalue{false};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

class rest_action : public gaction
{
public:
    rest_action() : gaction("Rest", 12)
    {
        gvalue health_precondition = 30.0f;
        gvalue health_effect = 100.0f;

        preconditions["health"] = gvalue{30.0f};
        effects["health"] = gvalue{100.0f};

        // Debug: verify types
        std::cout << "  [DEBUG] rest_action precondition type index: "
                  << health_precondition.index() << " (0=int, 1=bool, 2=float)\n";
        std::cout << "  [DEBUG] rest_action effect type index: "
                  << health_effect.index() << "\n";
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

class take_damage_action : public gaction
{
public:
    take_damage_action() : gaction("TakeDamage", 1)
    {
        preconditions["health"] = gvalue{100.0f};
        effects["health"] = gvalue{30.0f};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

class deep_action_1 : public gaction
{
public:
    deep_action_1() : gaction("DeepAction1", 1)
    {
        effects["depth_1"] = gvalue{1};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

class deep_action_2 : public gaction
{
public:
    deep_action_2() : gaction("DeepAction2", 1)
    {
        preconditions["depth_1"] = gvalue{1};
        effects["depth_2"] = gvalue{1};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

class deep_action_3 : public gaction
{
public:
    deep_action_3() : gaction("DeepAction3", 1)
    {
        preconditions["depth_2"] = gvalue{1};
        effects["depth_3"] = gvalue{1};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

class deep_action_4 : public gaction
{
public:
    deep_action_4() : gaction("DeepAction4", 1)
    {
        preconditions["depth_3"] = gvalue{1};
        effects["depth_4"] = gvalue{1};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

class deep_action_5 : public gaction
{
public:
    deep_action_5() : gaction("DeepAction5", 1)
    {
        preconditions["depth_4"] = gvalue{1};
        effects["depth_5"] = gvalue{1};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

/**
 * COMPLEX HEALTH SCENARIO ACTIONS
 * Scenario: Agent must navigate through dangerous area to reach healing
 */

/**
 * Cross a spike trap to reach the medpack room
 * Preconditions: at_entrance=true
 * Effects: at_medpack_room=true, at_entrance=false, health=10 (takes 10 damage)
 */
class cross_spike_trap_action : public gaction
{
public:
    cross_spike_trap_action() : gaction("CrossSpikeTrap", 15)
    {
        preconditions["at_entrance"] = gvalue{true};

        effects["at_entrance"] = gvalue{false};
        effects["at_medpack_room"] = gvalue{true};
        effects["health"] = gvalue{10.0f};  // Absolute value after taking damage
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

/**
 * Pick up the medpack from the medpack room
 * Preconditions: at_medpack_room=true, has_medpack=false
 * Effects: has_medpack=true
 */
class pickup_medpack_action : public gaction
{
public:
    pickup_medpack_action() : gaction("PickupMedpack", 3)
    {
        preconditions["at_medpack_room"] = gvalue{true};
        preconditions["has_medpack"] = gvalue{false};

        effects["has_medpack"] = gvalue{true};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

/**
 * Use the medpack to restore health to maximum
 * Preconditions: has_medpack=true
 * Effects: health=100.0f, has_medpack=false
 */
class use_medpack_action : public gaction
{
public:
    use_medpack_action() : gaction("UseMedpack", 5)
    {
        preconditions["has_medpack"] = gvalue{true};

        effects["health"] = gvalue{100.0f};
        effects["has_medpack"] = gvalue{false};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

/**
 * Alternative: Retreat back to entrance (if agent changes mind)
 * Preconditions: at_medpack_room=true
 * Effects: at_entrance=true, at_medpack_room=false
 */
class retreat_to_entrance_action : public gaction
{
public:
    retreat_to_entrance_action() : gaction("RetreatToEntrance", 20)
    {
        preconditions["at_medpack_room"] = gvalue{true};

        effects["at_entrance"] = gvalue{true};
        effects["at_medpack_room"] = gvalue{false};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};

/**
 * Emergency bandage - can be used anywhere but only heals to 50 HP
 * Preconditions: has_bandage=true
 * Effects: health=50.0f, has_bandage=false
 */
class use_bandage_action : public gaction
{
public:
    use_bandage_action() : gaction("UseBandage", 3)
    {
        preconditions["has_bandage"] = gvalue{true};

        effects["health"] = gvalue{50.0f};
        effects["has_bandage"] = gvalue{false};
    }

    bool setup() override { return true; }
    bool end() override { return true; }
};


#endif //IDAGOAP_TEST_ACTIONS_H