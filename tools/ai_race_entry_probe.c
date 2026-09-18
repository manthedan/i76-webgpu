/* Synthetic route-entry regression, NOT mission or original-executable proof.
 * P05's authored race -> sit -> race loop must enter from the current pose,
 * not rewind a stopped truck to node zero. See race-route-entry.md. */
#include <stdio.h>
#include "engine/ai.h"

static int failures;
static void check(int yes, const char *label)
{
    printf("%s %s\n", yes ? "ok  " : "FAIL", label);
    failures += !yes;
}

int main(void)
{
    const float path[] = {0,0,0, 100,0,0, 200,0,0, 300,0,0};
    ai_reset();
    ai_agent_init(1, 150, 0, 0);
    ai_race(1, 16, path, 4, 18, 1);
    check(ai_wp(1) == 2, "fresh race targets the forward node near the current pose");
    ai_sit(1);
    ai_sit(1);
    check(ai_goal(1) == AI_GOAL_NONE && ai_sat(1), "repeated sit remains parked");
    ai_race(1, 16, path, 4, 18, 1);
    check(ai_wp(1) == 2 && ai_race_active(1) && !ai_sat(1),
          "race after repeated sit resumes locally, not at node zero");
    ai_translate_xz(1, -160, 0);
    ai_race(1, 16, path, 4, 12, 1);
    check(ai_wp(1) == 2, "active race reissue does not reseed its cursor after contact");
    ai_sit(1);
    ai_race(1, 16, path, 4, 18, 1);
    check(ai_wp(1) == 0, "new entry before the course selects its first node");
    ai_sit(1);
    ai_translate_xz(1, 110, 0);
    ai_race(1, 16, path, 4, 18, 1);
    check(ai_wp(1) == 2, "entry exactly on a node selects its successor");
    ai_sit(1);
    ai_goto(1, 16, path, 4, 18);
    check(ai_wp(1) == 0 && !ai_race_active(1), "ordinary goto restart semantics are unchanged");
    ai_sit(1);
    const float point[] = {100,0,0};
    ai_race(1, 17, point, 1, 18, 1);
    check(ai_wp(1) == 0, "single-node race entry terminates its selection scan");
    ai_sit(1);
    const float repeated[] = {100,0,0, 100,0,0, 100,0,0};
    ai_race(1, 18, repeated, 3, 18, 1);
    check(ai_wp(1) == 0, "coincident nodes terminate at the first nearest tie");
    ai_reset();
    const float loop[] = {0,0,0, 100,0,0, 100,0,100, 0,0,100, 0,0,0};
    ai_agent_init(1, 0, 0, 0);
    ai_race(1, 19, loop, 5, 18, 1);
    for (int t = 0; t < 10000 && ai_arrivals(1) == 0; t++)
        ai_tick(AI_TICK_DT, NULL, NULL);
    check(ai_arrivals(1) == 1, "closed race completes its first traversal");
    ai_race(1, 19, loop, 5, 18, 1);
    for (int t = 0; t < 10; t++) ai_tick(AI_TICK_DT, NULL, NULL);
    check(ai_arrivals(1) == 1, "fresh lap cannot instantly arrive at the shared endpoint");
    for (int t = 0; t < 10000 && ai_arrivals(1) == 1; t++)
        ai_tick(AI_TICK_DT, NULL, NULL);
    check(ai_arrivals(1) == 2, "fresh lap traverses the closed race again");
    ai_reset();
    printf("RESULT: %s\n", failures ? "FAIL" : "PASS");
    return failures != 0;
}
