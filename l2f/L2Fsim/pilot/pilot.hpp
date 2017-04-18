#ifndef L2FSIM_PILOT_HPP_
#define L2FSIM_PILOT_HPP_

#include <vector>
#include <L2Fsim/aircraft/aircraft.hpp>

namespace L2Fsim {

class pilot {
public:
    virtual ~pilot() = default;

    /**
     * Apply the policy
     * @param {state &} s; reference on the state
     * @param {command &} u; reference on the command
     */
	virtual pilot& operator()(state &s, command &u) = 0;

    /**
     * Try to steer back the glider in the valid zone
     * @param {state &} s; reference on the state
     * @param {command &} u; reference on the command
     */
    virtual pilot& out_of_range(state &s, command &u) = 0;
};

}

#endif
