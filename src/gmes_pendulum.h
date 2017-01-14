#ifndef MAIN_APP_H_INCLUDED
#define MAIN_APP_H_INCLUDED

#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <unistd.h>

#include <ctime>
#include <cmath>
#include <cfloat>
#include <string>
#include <cerrno>
#include <clocale>
#include <sys/time.h>
#include <signal.h>
#include <vector>
#include <cassert>
#include <functional>

#include <common/application_interface.h>
#include <common/event_manager.h>
#include <common/log_messages.h>
#include <common/basic.h>
#include <common/gui.h>
#include <common/modules.h>
#include <common/setup.h>
#include <common/globalflag.h>
#include <common/static_vector.h>
#include <common/misc.h>
#include <common/vector_n.h>

#include <basic/color.h>

#include <robots/joint.h>
#include <robots/pole.h>

/* drawing */
#include <draw/draw.h>
#include <draw/axes.h>
#include <draw/axes3D.h>
#include <draw/plot1D.h>
#include <draw/plot3D.h>
#include <draw/network3D.h>

#include <control/spaces.h>
#include <control/sensorspace.h>
#include <control/jointcontrol.h>
#include <control/controlparameter.h>

#include <learning/gmes.h>
#include <learning/gmes_graphics.h>
#include <learning/sarsa.h>
#include <learning/sarsa_graphics.h>
#include <learning/reward.h>
#include <learning/eigenzeit.h>
#include <learning/payload.h>
#include <learning/payload_graphics.h>
#include <learning/competitive_motor_layer.h>
#include <learning/competitive_motor_layer_graphics.h>
#include <learning/action_module.h>
#include <learning/action_selection.h>
#include <learning/epsilon_greedy.h>
#include <learning/boltzmann_softmax.h>

/**TODO solve the problem
 * - of falling on back (robots)
 * - spinning around (pendulum)
 * - clustering of experts
 * !11111elf */

/**TODO
 * make use of sdl gfx for drawing basic shapes
 * 'Learning strategy'
 */

/**die metrik nach derer eine action gelöscht werden kann ist schlecht.
* es führt dazu, dass seltener gewählte aktionen aussterben, obwohl sie wichtig sind.*/


namespace constants { /**TODO make these constants experiment settings*/
    /*gmes*/
    const std::size_t number_of_experts       = 225;
    const std::size_t max_number_of_actions   = 4;
    const std::size_t number_of_actions_begin = 4;
    const double      gmes_learning_rate      = 35.0; // global learning rate
    const double      local_learning_rate     = 0.005; // 0.005
    const std::size_t experience_size         = 1;

    /*eigenzeit*/
    const std::size_t eigenzeit_steps         = 100;

    /*trials*/
    const VectorN     trial_durations         = {1000, 2000, 500};

    /*sarsa*/
    const VectorN     sarsa_learning_rates    = {0.5, 0.1, 0.1};
    const double      epsilon_exploration     = 0.05;
    const double      initial_qvalue          = 10.0; /**optimistic initialization, if reward is normalized between 0..1 */

    /*control*/
    const bool        self_adjusting          = false;
    const double      mutation_rate           = 0.0001; // 0.001
    const double      learning_rate           = 0.0010; // 0.01
    const control::Minimal_Seed_t& seed       = {0.,0.,0.};
}

class Application : public Application_Interface, public Application_Base
{
public:

    Application(int argc, char **argv, Event_Manager &em)
    : Application_Base("GMES Pendulum", 800, 800)
    , event(em)
    , cycles(0)
    , robot(true)
    , parameter_set(constants::max_number_of_actions, "../data/seeds/ctrl_pendulum/")
    , sensors(robot.get_joints())
    , control( robot
             , parameter_set
             , constants::max_number_of_actions
             , constants::number_of_actions_begin
             , payloads
             , sarsa
             , constants::self_adjusting
             , constants::mutation_rate
             , constants::learning_rate
             , constants::seed )
    , reward(gmes, robot.get_joints())
    , payloads(constants::number_of_experts, control, reward.get_number_of_policies(), constants::initial_qvalue)
    , experts(constants::number_of_experts, payloads, sensors, constants::local_learning_rate, gmes_constants::random_weight_range, constants::experience_size)
    , gmes(experts, constants::gmes_learning_rate)
    , epsilon_greedy(payloads, control, constants::epsilon_exploration)
    , boltzmann_softmax(payloads, control, constants::epsilon_exploration)
    , sarsa(payloads, reward, epsilon_greedy, control.get_number_of_actions(), constants::sarsa_learning_rates)
    , policy_selector(sarsa, reward.get_number_of_policies(), true)
    , eigenzeit(gmes, constants::eigenzeit_steps)
    , table(3)
    /* graphics */
    , control_graphics(control)
    , gmes_graphics(gmes, sensors)
    , sarsa_graphics(sarsa)
    , payload_graphics(gmes, gmes_graphics, payloads, sarsa)
    , ext_payload_graphics(payloads)
    , policy_selector_graphics(policy_selector)
    , eigenzeit_graphics(eigenzeit)
    {
        assert(sensors.size() == 3);

        /* register key event */
        Event_Manager::callback_type callback = std::bind(&Application::user_callback_keyboard, this, std::placeholders::_1);
        event.register_user_callback_key_pressed(callback);

        /**TODO where to place this?*/
        assert(constants::trial_durations.size() == policy_selector.size());
        for (std::size_t i = 0; i < policy_selector.size(); ++i)
            policy_selector.set_policy_trial_duration(i, constants::trial_durations[i]);

        /**TODO: positions of the drawings */
        control_graphics        .set_position(0.0, 0.0).set_scale(1.0);
        gmes_graphics           .set_position(0.0, 0.0).set_scale(1.0); // done
        sarsa_graphics          .set_position(0.0, 0.0).set_scale(1.0);
        payload_graphics        .set_position(0.0, 0.0).set_scale(1.0);
        ext_payload_graphics    .set_position(0.0,-2.0).set_scale(1.0);
        policy_selector_graphics.set_position(0.0, 0.0).set_scale(1.0);
        eigenzeit_graphics      .set_position(0.0, 0.0).set_scale(1.0);
    }

    bool loop();
    void finish();
    void draw(const pref& p) const;

    bool     visuals_enabled(void)       { return true;   }
    uint64_t get_cycle_count(void) const { return cycles; }
    void     user_callback_keyboard(const SDL_Keysym &keysym);

private:
    Event_Manager&                         event;
    uint64_t                               cycles;

    robots::pole                           robot;
    control::Control_Vector                parameter_set;

    control::pendulum_sensor_space         sensors;
    control::self_adjusting_motor_space    control;
    control::pendulum_reward_space         reward;
    static_vector<State_Payload>           payloads;
    Expert_Vector                          experts;

    GMES                                   gmes;

    Epsilon_Greedy                         epsilon_greedy; /**TODO integrate into sarsa, as template parameter, with eps (init) also as template param*/
    Boltzmann_Softmax                      boltzmann_softmax;
    SARSA                                  sarsa;
    Policy_Selector                        policy_selector;

    Eigenzeit                              eigenzeit;
    ColorTable                             table;

    /* graphics */
    control::self_adjusting_motor_space_graphics      control_graphics;
    GMES_Graphics                                        gmes_graphics;
    SARSA_Graphics                                      sarsa_graphics;
    Payload_Graphics                                  payload_graphics;
    State_Payload_Graphics                        ext_payload_graphics;
    Policy_Selector_Graphics                  policy_selector_graphics;
    Eigenzeit_Graphics                              eigenzeit_graphics;
};

#endif // MAIN_APP_H_INCLUDED
