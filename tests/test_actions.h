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

#endif //IDAGOAP_TEST_ACTIONS_H