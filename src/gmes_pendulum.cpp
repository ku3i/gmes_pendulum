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

        case SDLK_l: motor_layer.toggle_learning(); break;
        default:
            break;
    }

    views.key_pressed(keysym);
}

void
Application::draw(const pref& p) const
{
    robot.draw(0.0, 0.0, 2.0);

    gfx_gmes           .drawing(p);
    gfx_policy_selector.drawing(p);
    gfx_eigenzeit      .drawing(p);
    gfx_payload        .drawing(p);
    if (views.get() == 1) {
        gfx_agent      .drawing(p);
        gfx_ext_payload.drawing(p);
    }
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

    robot.execute_cycle();
    controller.execute_cycle();
    sensors.execute_cycle();
    motor_layer.execute_cycle();

    gmes.execute_cycle();
    gfx_gmes.execute_cycle(cycles);

    eigenzeit.execute_cycle();

    reward.execute_cycle();
    policy_selector.execute_cycle();

    if (eigenzeit.has_progressed())
        agent.execute_cycle(gmes.get_winner());

    gfx_agent.execute_cycle(cycles, eigenzeit.has_progressed());

    if (eigenzeit.has_progressed() /**TODO: and actions.has_current_selection_changed()*/)
        controller.set_control_parameter(actions.get_controller_weights(agent.get_current_action())); //note: must be processed after sarsa.

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
