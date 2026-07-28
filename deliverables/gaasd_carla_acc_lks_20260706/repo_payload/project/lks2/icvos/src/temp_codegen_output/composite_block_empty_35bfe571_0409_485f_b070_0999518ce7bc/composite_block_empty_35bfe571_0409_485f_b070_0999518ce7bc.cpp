#include "composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bc.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bc::run(const Input &input, Output &output)
{
    
    
    Real temp359918;
    Real temp360555;
    Real temp360910;
    Real temp361179;
    Real temp361412;
    
    
    /* Fabs_Steer */
    temp359918 = fabs(input.driverSteerNorm);

    temp360555 = input.egoV >= global::params.lks_vMin;
    temp361179 = temp359918 >= global::params.lks_driverSteerThreshold;
    temp360910 = !temp360555;
    temp361412 = input.brakePressed || temp361179 || temp360910;
    output.controlEnabled = !temp361412;
    output.controlEnabled = output.controlEnabled;

    
}
} // namespace control
