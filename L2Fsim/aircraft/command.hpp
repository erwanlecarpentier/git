#ifndef L2FSIM_COMMAND_HPP_
#define L2FSIM_COMMAND_HPP_

#include <vector>

/**
 * The abstract class state providing general access to the command of an aircraft
 * @version 1.0
 * @since 1.0
 */

namespace L2Fsim {

class command {
public:
    virtual ~command() = default;

    /** @deprecated Get a reference on the command */
	//virtual command & get_command() = 0;

    /** Set the command */
	virtual void set_command(command &_u) = 0;

	virtual void set_to_neutral() = 0;
};

}

#endif
