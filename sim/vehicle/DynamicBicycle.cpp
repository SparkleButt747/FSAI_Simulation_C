#include "DynamicBicycle.hpp"
#include <yaml.h>
#include <stdexcept>

using std::cos;
using std::sin;
using std::atan;
using std::tan;
using std::max;
using std::min;

namespace {
inline double clamp(double v, double lo, double hi){return v<lo?lo:(v>hi?hi:v);} // helper
inline double clamp01(double v){return v<0?0:(v>1?1:v);}                     // 0..1
}

// ---------------- VehicleModel ------------------

void VehicleModel::updateState(VehicleState&, const VehicleInput&, double) {
    // Base class does nothing
}

void VehicleModel::validateState(VehicleState& state) const {
    state.velocity.x() = std::max(0.0, state.velocity.x());
}

void VehicleModel::validateInput(VehicleInput& input) const {
    input.acc   = clamp(input.acc,   param_.input_ranges.acc.min,   param_.input_ranges.acc.max);
    input.vel   = clamp(input.vel,   param_.input_ranges.vel.min,   param_.input_ranges.vel.max);
    input.delta = clamp(input.delta, param_.input_ranges.delta.min, param_.input_ranges.delta.max);
}

double VehicleModel::getSlipAngle(const VehicleState& x, const VehicleInput& u, bool isFront) const {
    double leverArmLength = param_.kinematic.l * param_.kinematic.w_front;
    if(!isFront){
        double vX = std::max(1.0, x.velocity.x());
        return atan((x.velocity.y() - leverArmLength * x.rotation.z()) / (vX - 0.5 * param_.kinematic.axle_width * x.rotation.z()));
    } else {
        double vXFront = std::max(1.0, x.velocity.x());
        return atan((x.velocity.y() + leverArmLength * x.rotation.z()) / (vXFront - 0.5 * param_.kinematic.axle_width * x.rotation.z())) - u.delta;
    }
}

WheelsInfo VehicleModel::getWheelSpeeds(const VehicleState& state, const VehicleInput& input) const {
    WheelsInfo wheelSpeeds = WheelsInfo_default();
    double wheelCircumference = 2.0 * M_PI * param_.tire.radius;
    double rpm = (state.velocity.x() / wheelCircumference) * 60.0;
    wheelSpeeds.lf_speed = wheelSpeeds.rf_speed = wheelSpeeds.lb_speed = wheelSpeeds.rb_speed = static_cast<float>(rpm);
    wheelSpeeds.steering = static_cast<float>(input.delta);
    return wheelSpeeds;
}

// ---------------- DynamicBicycle ------------------

double DynamicBicycle::calculateMagnitude(double x, double y){
    return std::sqrt(x*x + y*y);
}

DynamicBicycle::Forces DynamicBicycle::computeForces(const VehicleState& x, const VehicleInput& u) const {
    Forces f{0.0,0.0,0.0};
    double vx = x.velocity.x();
    double Fdrag = param_.aero.c_drag * vx * vx;
    double Fdown = param_.aero.c_down * vx * vx;
    double Fz = param_.inertia.g * param_.inertia.m + Fdown;

    double slipAngleFront = getSlipAngle(x,u,true);
    double slipAngleRear  = getSlipAngle(x,u,false);

    auto getFy = [&](double FzTotal, bool front, double slip){
        double FzAxle = front ? 0.5 * param_.kinematic.w_front * FzTotal
                              : 0.5 * (1.0 - param_.kinematic.w_front) * FzTotal;
        double B = param_.tire.B;
        double C = param_.tire.C;
        double D = param_.tire.D;
        double E = param_.tire.E;
        double muY = D * sin(C * atan(B * (1.0 - E) * slip + E * atan(B * slip)));
        return FzAxle * muY;
    };

    f.FyF = getFy(Fz,true,slipAngleFront);
    f.FyR = getFy(Fz,false,slipAngleRear);
    double acc = (vx <= 0.0 && u.acc < 0.0) ? 0.0 : u.acc;
    f.Fx = acc * param_.inertia.m - Fdrag;
    return f;
}

static VehicleState fDynamics(const DynamicBicycle& db, const VehicleState& x, const VehicleInput& u, double Fx, double FyF, double FyR){
    VehicleState xDot;
    double FyFTot = 2.0 * FyF;
    double FyRTot = 2.0 * FyR;
    xDot.position.x() = cos(x.yaw) * x.velocity.x() - sin(x.yaw) * x.velocity.y();
    xDot.position.y() = sin(x.yaw) * x.velocity.x() + cos(x.yaw) * x.velocity.y();
    xDot.yaw = x.rotation.z();
    xDot.velocity.x() = (x.rotation.z() * x.velocity.y() + (Fx - sin(u.delta) * FyFTot)) / db.param().inertia.m;
    xDot.velocity.y() = ((cos(u.delta) * FyFTot) + FyRTot) / db.param().inertia.m - (x.rotation.z() * x.velocity.x());
    xDot.rotation.z() = (cos(u.delta) * FyFTot * db.param().kinematic.l_F - FyRTot * db.param().kinematic.l_R) / db.param().inertia.I_z;
    return xDot;
}

static VehicleState fKinematicCorrection(const DynamicBicycle& db, const VehicleState& xIn, const VehicleState& xState, const VehicleInput& u, double Fx, double dt){
    VehicleState x = xIn;
    double vXDot = Fx / db.param().inertia.m;
    double v = DynamicBicycle::calculateMagnitude(xState.velocity.x(), xState.velocity.y());
    double vBlend = 0.5 * (v - 1.5);
    double blend = clamp01(vBlend);

    x.velocity.x() = blend * x.velocity.x() + (1.0 - blend) * (xState.velocity.x() + dt * vXDot);
    double vY = tan(u.delta) * x.velocity.x() * db.param().kinematic.l_R / db.param().kinematic.l;
    double r  = tan(u.delta) * x.velocity.x() / db.param().kinematic.l;
    x.velocity.y() = blend * x.velocity.y() + (1.0 - blend) * vY;
    x.rotation.z() = blend * x.rotation.z() + (1.0 - blend) * r;
    return x;
}

void DynamicBicycle::updateState(VehicleState& state, const VehicleInput& inputRaw, double dt) {
    VehicleInput input = inputRaw;
    validateInput(input);
    Forces forces = computeForces(state, input);
    VehicleState xDot = fDynamics(*this, state, input, forces.Fx, forces.FyF, forces.FyR);
    VehicleState xNext = state + xDot * dt;
    VehicleState corrected = fKinematicCorrection(*this, xNext, state, input, forces.Fx, dt);
    state = corrected;
    state.acceleration.x() = xDot.velocity.x();
    state.acceleration.y() = xDot.velocity.y();
    state.acceleration.z() = 0.0;
    validateState(state);
}
