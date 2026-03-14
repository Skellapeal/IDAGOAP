//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "transposition_table.h"

using namespace rida_goap;

TEST(TranspositionTable, LookupMissReturnsNullopt)
{
    transposition_table tt;
    world_state ws;
    ws.set_bool("x", true);
    EXPECT_FALSE(tt.lookup(ws).has_value());
}

TEST(TranspositionTable, StoreAndLookupReturnsStoredCost)
{
    transposition_table tt;
    world_state ws;
    ws.set_bool("x", true);
    tt.store(ws, 5);
    const auto result = tt.lookup(ws);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 5);
}

TEST(TranspositionTable, OverwriteUpdatesValue)
{
    transposition_table tt;
    world_state ws;
    ws.set_int("ammo", 3);
    tt.store(ws, 10);
    tt.store(ws, 2);
    EXPECT_EQ(*tt.lookup(ws), 2);
}

TEST(TranspositionTable, ClearEmptiesTable)
{
    transposition_table tt;
    world_state ws;
    ws.set_bool("alive", true);
    tt.store(ws, 1);
    tt.clear();
    EXPECT_EQ(tt.size(), 0u);
    EXPECT_FALSE(tt.lookup(ws).has_value());
}

TEST(TranspositionTable, SizeTracksEntries)
{
    transposition_table tt;
    world_state a, b;
    a.set_bool("x", true);
    b.set_bool("y", true);
    tt.store(a, 1);
    tt.store(b, 2);
    EXPECT_EQ(tt.size(), 2u);
}

TEST(TranspositionTable, LRUEvictsWhenFull)
{
    transposition_table tt;
    tt.set_max_size(2);

    world_state a, b, c;
    a.set_int("k", 1);
    b.set_int("k", 2);
    c.set_int("k", 3);

    tt.store(a, 10);
    tt.store(b, 20);
    tt.store(c, 30);

    EXPECT_FALSE(tt.lookup(a).has_value());
    EXPECT_TRUE(tt.lookup(b).has_value());
    EXPECT_TRUE(tt.lookup(c).has_value());
    EXPECT_EQ(tt.size(), 2u);
}