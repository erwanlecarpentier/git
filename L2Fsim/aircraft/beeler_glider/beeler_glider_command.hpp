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

    /** @brief Set the command to neutral */
    void set_to_neutral() override {
        dalpha = dbeta = dsigma = 0.;
    }

    /** @brief Return true if this action is equal to the argument */
    bool equals(const beeler_glider_command &a) {
        if(is_equal_to(dalpha,a.dalpha)
        && is_equal_to(dbeta,a.dbeta)
        && is_equal_to(dsigma,a.dsigma)) {
            return true;
        } else {
            return false;
        }
    }
};

}

#endif
