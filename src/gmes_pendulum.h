/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | July 2017                       |
 +---------------------------------*/

#include <common/application_base.h>
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
#include <common/view_manager.h>

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
#include <learning/gmes_action_module.h>
#include <learning/action_selection.h>
#include <learning/epsilon_greedy.h>
#include <learning/boltzmann_softmax.h>
#include <learning/motor_layer.h>

/**TODO solve the problem
 * - of falling on back (robots)
 * - spinning around (pendulum)
 * - clustering of experts
 * !11111elf */

/**TODO
 * make use of sdl gfx for drawing basic shapes
 * 'Learning strategy'
 */

namespace constants { /**TODO make these constants experiment settings*/
    /*gmes*/
    const std::size_t number_of_experts       = 225;
    const std::size_t max_number_of_actions   = 20;
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

    /* motor control*/
    //const double      learning_rate           = 0.;
    //const double      growth_rate             = 0.;
    //const std::size_t experience_size         = 1;
    //const control::Minimal_Seed_t& seed       = {0.,0.,0.};
}

class Application : public Application_Base
{
public:

    Application(int argc, char** argv, Event_Manager &em)
    : Application_Base(argc, argv, em, "GMES Pendulum", 800, 800)
    , robot(true)
    , controller(robot)
    , sensors(robot.get_joints())
    , motor_layer( robot
                 , constants::max_number_of_actions/**TODO increase*/
                 , 0.001 /*learning_rate*/
                 , 1.0   /*growth rate*/
                 , 1 /*experience size */
                 , 0.005 /*noise level*/
                 , "../data/seeds/ctrl_pendulum/"
                 , {0.,0.,0.} /*minimal seed*/
                 , constants::number_of_actions_begin )
    , actions(motor_layer)
    , reward(gmes, robot.get_joints())
    , payloads(constants::number_of_experts, actions, reward.get_number_of_policies(), constants::initial_qvalue)
    , experts(constants::number_of_experts, payloads, sensors, constants::local_learning_rate, gmes_constants::random_weight_range, constants::experience_size)
    , gmes(experts, constants::gmes_learning_rate, /*oneshot=*/true, /*initial_experts=*/1, "state")
    , epsilon_greedy(payloads, actions, constants::epsilon_exploration)
    , boltzmann_softmax(payloads, actions, constants::epsilon_exploration)
    , agent(payloads, reward, epsilon_greedy, actions.get_number_of_actions(), constants::sarsa_learning_rates)
    , policy_selector(agent, reward.get_number_of_policies(), true)
    , eigenzeit(gmes, constants::eigenzeit_steps)
    , table(3)
    /* graphics */
    , gfx_gmes(gmes, sensors)
    , gfx_agent(agent)
    , gfx_payload(gmes, gfx_gmes, payloads, agent)
    , gfx_ext_payload(payloads, actions)
    , gfx_policy_selector(policy_selector)
    , gfx_eigenzeit(eigenzeit)
    , views(2)
    {
        assert(sensors.size() == 3);

        /**TODO where to place this?*/
        assert(constants::trial_durations.size() == policy_selector.size());
        for (std::size_t i = 0; i < policy_selector.size(); ++i)
            policy_selector.set_policy_trial_duration(i, constants::trial_durations[i]);

        /**TODO: positions of the drawings */
        gfx_gmes           .set_position(0.0, 0.0).set_scale(1.0); // done
        gfx_agent          .set_position(0.0, 0.0).set_scale(1.0);
        gfx_payload        .set_position(0.0, 0.0).set_scale(1.0);
        gfx_ext_payload    .set_position(0.0,-2.0).set_scale(1.0);
        gfx_policy_selector.set_position(0.0, 0.0).set_scale(1.0);
        gfx_eigenzeit      .set_position(0.0, 0.0).set_scale(1.0);


        /**TODO: remove */
        motor_layer.enable_learning(false);
    }

    bool loop();
    void finish();
    void draw(const pref& p) const;
    void user_callback_key_pressed (const SDL_Keysym& keysym);

private:
    robots::pole                           robot;
    control::Jointcontrol                  controller;
    control::pendulum_sensor_space         sensors;

    learning::Motor_Layer                  motor_layer;
    learning::gmes_action_module           actions;
    control::pendulum_reward_space         reward;      //TODO: move to separate file and learning namespace
    static_vector<State_Payload>           payloads;
    Expert_Vector                          experts;

    GMES                                   gmes;

    learning::Epsilon_Greedy               epsilon_greedy; /**TODO integrate into sarsa, as template parameter, with eps (init) also as template param*/
    Boltzmann_Softmax                      boltzmann_softmax;
    SARSA                                  agent;
    Policy_Selector                        policy_selector;

    learning::Eigenzeit                    eigenzeit;
    ColorTable                             table;

    /* graphics */
    GMES_Graphics                          gfx_gmes;
    SARSA_Graphics                         gfx_agent;
    Payload_Graphics                       gfx_payload;
    State_Payload_Graphics                 gfx_ext_payload;
    Policy_Selector_Graphics               gfx_policy_selector;
    learning::Eigenzeit_Graphics           gfx_eigenzeit;

    View_Manager                           views;
};
