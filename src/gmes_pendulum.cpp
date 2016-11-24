/*
 * Matthias Kubisch
 * kubisch@informatik.hu-berlin.de
 * created 17.Okt 2014
 */

#include "./gmes_pendulum.h"

DEFINE_GLOBALS()

void
Application::user_callback_keyboard(const SDL_Keysym &keysym)
{
    switch (keysym.sym)
    {
        case SDLK_TAB: policy_selector.toggle_random_policy_mode(); break;

        case SDLK_1: policy_selector.select_policy(0); break;
        case SDLK_2: policy_selector.select_policy(1); break;
        case SDLK_3: policy_selector.select_policy(2); break;

        default:
            break;
    }
}

void
Application::draw(const pref& p) const
{
    robot.draw(0.0, 0.0, 2.0);

    gmes_graphics           .drawing(p);
    sarsa_graphics          .drawing(p);
    control_graphics        .drawing(p);
    policy_selector_graphics.drawing(p);
    eigenzeit_graphics      .drawing(p);
    payload_graphics        .drawing(p);
    ext_payload_graphics    .drawing(p);
}

bool
Application::loop(void)
{
    /** maybe we have to increase the randomness (exploration) when using eigenzeit.
    random decisions happen less often with eigenzeit, answer: no 10% is 10% no matter of which time step size is being used!
    but maybe its the learning rate, learning also takes place less often
    but maybe we do not have to do anything */

    /** THINK: every individual needs multiple fitness values, in the number of policies.
     * sort by fitness must choose the appropriate fitness value, e.g. needs to know the no. of policies
     * need a new type of individual (maybe as a template)
     * need a template version of population */

    /** to further evolve an action with the pool-based strategy it is assumed that every individual (each trial) has (almost)
    the same preconditions, i.e. is in the same state. Can this be reached with having only a single list of actions to share
    or is it mandatory that every state carries its own list of actions.*/

    /** Think about a learning schedule, for determining the duration and order of learning tasks,
    instead of randomly choosing a policy to follow:
    duration is now configurable, schedule not yet */

    /**TODO apply action means to choose a new setting for joint controller sarsa should have a reference to the jointcontroller
    (implementing the action module) */

    /** TODO: integer learning capacity with discrete decrement steps for GMES*/

    robot.execute_cycle();
    sensors.execute_cycle();

    gmes.execute_cycle();
    gmes_graphics.execute_cycle(cycles);

    eigenzeit.execute_cycle();

    reward.execute_cycle();
    policy_selector.execute_cycle();

    if (eigenzeit.has_progressed())
        sarsa.execute_cycle(gmes.get_winner());

    sarsa_graphics.execute_cycle(cycles, eigenzeit.has_progressed());

    control         .execute_cycle(eigenzeit.has_progressed()); //note: must be processed after sarsa.
    control_graphics.execute_cycle(eigenzeit.has_progressed());

    if (eigenzeit.has_progressed())
        reward.clear_aggregations();

    ++cycles;
    return true;
}

void
Application::finish(void)
{
    sts_msg("Finished shutting down all subsystems.");
    quit();
}

APPLICATION_MAIN()
