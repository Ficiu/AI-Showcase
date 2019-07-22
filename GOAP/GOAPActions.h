#pragma once

/**
 *	GOAPActions.h
 *
 *	A class for each action, with different preconditions and effects
 *
 */

#include "GOAPAction.h"
#include "WorldState.h"
#include "Attack.h"
#include "FollowPath.h"
#include "Build.h"
#include "Gather.h"


namespace goap
{

    class goap_attack : public GOAPAction
    {

    public:

        goap_attack();

    };

    inline goap_attack::goap_attack()
    {
        action_name_ = "attack";
        
        SetPrecondition(Data::states_types::target_acquired, true);
        SetEffect(Data::states_types::target_dead, true);
        SetActionState(new Attack<Unit>);

    }


    class goap_build : public GOAPAction
    {

    public:

        goap_build();

    };

    inline goap_build::goap_build()
    {
        action_name_ = "build";

        SetPrecondition(Data::states_types::resources_for_build_x, true);
        SetEffect(Data::states_types::build_built, true);
        SetEffect(Data::states_types::resources_for_build_x, false);
        SetActionState(new Build<Unit>);

    }


    class goap_move_to : public GOAPAction
    {

    public:

        goap_move_to();

    };

    inline goap_move_to::goap_move_to()
    {
        action_name_ = "move_to";

        SetPrecondition(Data::states_types::target_acquired, false);
        SetEffect(Data::states_types::target_acquired, true);
        SetActionState(new FollowPath<Unit>);

    }


    class goap_explore_for_resource : public GOAPAction
    {

    public:

        goap_explore_for_resource();

    };

    inline goap_explore_for_resource::goap_explore_for_resource()
    {
        action_name_ = "explore_for_resource";

        SetPrecondition(Data::states_types::resource_available, false);
        SetEffect(Data::states_types::resource_available, true);
        SetActionState(new FollowPath<Unit>);

    }


    class goap_explore_for_enemy : public GOAPAction
    {

    public:

        goap_explore_for_enemy();

    };

    inline goap_explore_for_enemy::goap_explore_for_enemy()
    {
        action_name_ = "explore_for_enemy";

        SetPrecondition(Data::states_types::enemy_in_range, false);
        SetEffect(Data::states_types::enemy_in_range, true);
        SetActionState(new FollowPath<Unit>);

    }


    class goap_gather_resource : public GOAPAction
    {

    public:

        goap_gather_resource();

    };

    inline goap_gather_resource::goap_gather_resource()
    {
        action_name_ = "gather_resource";

        SetPrecondition(Data::states_types::resource_available, true);
        SetEffect(Data::states_types::resources_for_build_x, true);
        SetActionState(new Gather<Unit>);

    }

}
