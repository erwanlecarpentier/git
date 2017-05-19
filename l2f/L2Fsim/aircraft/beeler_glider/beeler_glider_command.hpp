#ifndef L2FSIM_BEELER_GLIDER_COMMAND_HPP_
#define L2FSIM_BEELER_GLIDER_COMMAND_HPP_

#include <L2Fsim/aircraft/command.hpp>

/**
 * Beeler's glider command
 * @version 1.0
 * @since 1.0
 */

namespace L2Fsim {

class beeler_glider_command : public command {
public:
    /**
     * @brief Attributes
     * @param {double} dalpha; variation of angle of attack
     * @param {double} dbeta; variation of sideslip angle
     * @param {double} dsigma; variation of bank angle
     */
    double dalpha, dbeta, dsigma;

    /** @brief Constructor */
    beeler_glider_command(double _dalpha=0., double _dbeta=0., double _dsigma=0.) :
        dalpha(_dalpha),
        dbeta(_dbeta),
        dsigma(_dsigma)
    {}

    /** @brief Set the command */
    void set_command(command &_u) override {
        beeler_glider_command &u = dynamic_cast <beeler_glider_command &> (_u);
        dalpha = u.dalpha;
        dbeta = u.dbeta;
        dsigma = u.dsigma;
    }
};

}

#endif
