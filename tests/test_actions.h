//
// Created by Niall Ó Colmáin on 04/03/2026.
//

#ifndef IDAGOAP_TEST_ACTIONS_H
#define IDAGOAP_TEST_ACTIONS_H

#include "../goap_action.h"
#include "../goap_types.h"

/**
 * Test action: Gather wood from the environment
 * Preconditions: none
 * Effects: has_wood = 1
 */
class gather_wood_action : public goap_action
{
public:
    gather_wood_action() : goap_action("GatherWood", 10)
    {
        add_effect("has_wood", 1);
    }
};

/**
 * Test action: Craft a tool using wood
 * Preconditions: has_wood = 1
 * Effects: has_tool = 1, has_wood = 0 (consumes wood)
 */
class craft_tool_action : public goap_action
{
public:
    craft_tool_action() : goap_action("CraftTool", 5)
    {
        add_precondition("has_wood", gvalue{1}, gcomparison::Equal);

        add_effect("has_tool", 1);
        add_effect("has_wood", 0);
    }
};

/**
 * Test action: Gather stone from the environment
 * Preconditions: none
 * Effects: has_stone = 1
 */
class gather_stone_action : public goap_action
{
public:
    gather_stone_action() : goap_action("GatherStone", 15)
    {
        add_effect("has_stone", 1);
    }
};

/**
 * Test action: Build shelter using wood and stone
 * Preconditions: has_wood = 1, has_stone = 1
 * Effects: has_shelter = 1, has_wood = 0, has_stone = 0
 */
class build_shelter_action : public goap_action
{
public:
    build_shelter_action() : goap_action("BuildShelter", 20)
    {
        add_precondition("has_wood", gvalue{1}, gcomparison::Equal);
        add_precondition("has_stone", gvalue{1}, gcomparison::Equal);

        add_effect("has_shelter", 1);
        add_effect("has_wood", 0);
        add_effect("has_stone", 0);
    }
};

/**
 * Test action: An action that's conditionally disabled
 * Used to test can_run() filtering
 */
class disabled_action : public goap_action
{
public:
    disabled_action() : goap_action("DisabledAction", 1)
    {
        add_effect("should_not_use", 1);
    }

    bool can_run() const override { return false; }
};

class scout_action : public goap_action
{
public:
    scout_action() : goap_action("Scout", 8)
    {
        add_effect("enemy_visible", true);
    }
};

class hide_action : public goap_action
{
public:
    hide_action() : goap_action("Hide", 3)
    {
        add_precondition("enemy_visible", gvalue{true}, gcomparison::Equal);

        add_effect("is_hidden", true);
        add_effect("enemy_visible", false);
    }
};

class rest_action : public goap_action
{
public:
    rest_action() : goap_action("Rest", 12)
    {
        add_precondition("health", gvalue{30.0f}, gcomparison::LessOrEqual);
        add_effect("health", 100.0f);
    }
};

class take_damage_action : public goap_action
{
public:
    take_damage_action() : goap_action("TakeDamage", 1)
    {
        add_precondition("health", gvalue{100.0f}, gcomparison::Equal);
        add_effect("health", 30.0f);
    }
};

class deep_action_1 : public goap_action
{
public:
    deep_action_1() : goap_action("DeepAction1", 1)
    {
        add_effect("depth_1", 1);
    }
};

class deep_action_2 : public goap_action
{
public:
    deep_action_2() : goap_action("DeepAction2", 1)
    {
        add_precondition("depth_1", gvalue{1}, gcomparison::Equal);
        add_effect("depth_2", 1);
    }
};

class deep_action_3 : public goap_action
{
public:
    deep_action_3() : goap_action("DeepAction3", 1)
    {
        add_precondition("depth_2", gvalue{1}, gcomparison::Equal);
        add_effect("depth_3", 1);
    }
};

class deep_action_4 : public goap_action
{
public:
    deep_action_4() : goap_action("DeepAction4", 1)
    {
        add_precondition("depth_3", gvalue{1}, gcomparison::Equal);
        add_effect("depth_4", 1);;
    }
};

class deep_action_5 : public goap_action
{
public:
    deep_action_5() : goap_action("DeepAction5", 1)
    {
        add_precondition("depth_4", gvalue{1}, gcomparison::Equal);
        add_effect("depth_5", 1);
    }
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
class cross_spike_trap_action : public goap_action
{
public:
    cross_spike_trap_action() : goap_action("CrossSpikeTrap", 15)
    {
        add_precondition("at_entrance", gvalue{true}, gcomparison::Equal);

        add_effect("at_entrance", false);
        add_effect("at_medpack_room", true);
        add_effect("health", 10.0f);
    }
};

/**
 * Pick up the medpack from the medpack room
 * Preconditions: at_medpack_room=true, has_medpack=false
 * Effects: has_medpack=true
 */
class pickup_medpack_action : public goap_action
{
public:
    pickup_medpack_action() : goap_action("PickupMedpack", 3)
    {
        add_precondition("at_medpack_room", gvalue{true},gcomparison::Equal);
        add_precondition("has_medpack", gvalue{false},gcomparison::Equal);

        add_effect("has_medpack", true);
    }
};

/**
 * Use the medpack to restore health to maximum
 * Preconditions: has_medpack=true
 * Effects: health=100.0f, has_medpack=false
 */
class use_medpack_action : public goap_action
{
public:
    use_medpack_action() : goap_action("UseMedpack", 5)
    {
        add_precondition("has_medpack", gvalue{true},gcomparison::Equal);

        add_effect("health", 100.0f);
        add_effect("has_medpack", false);
    }
};

/**
 * Alternative: Retreat back to entrance (if agent changes mind)
 * Preconditions: at_medpack_room=true
 * Effects: at_entrance=true, at_medpack_room=false
 */
class retreat_to_entrance_action : public goap_action
{
public:
    retreat_to_entrance_action() : goap_action("RetreatToEntrance", 20)
    {
        add_precondition("at_medpack_room", gvalue{true}, gcomparison::Equal);

        add_effect("at_entrance", true);
        add_effect("at_medpack_room", false);
    }
};

/**
 * Emergency bandage - can be used anywhere but only heals to 50 HP
 * Preconditions: has_bandage=true
 * Effects: health=50.0f, has_bandage=false
 */
class use_bandage_action : public goap_action
{
public:
    use_bandage_action() : goap_action("UseBandage", 3)
    {
        add_precondition("has_bandage", gvalue{true}, gcomparison::Equal);

        add_effect("health", 50.0f);
        add_effect("has_bandage", false);
    }
};

#endif //IDAGOAP_TEST_ACTIONS_H