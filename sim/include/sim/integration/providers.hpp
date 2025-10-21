#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/types.h"
#include "Vector.h"
#include "VehicleState.hpp"
#include "sim/integration/path_truth.hpp"

namespace fsai {
namespace integration {

class IVisionProvider {
public:
    virtual ~IVisionProvider() = default;
    virtual const std::string& Name() const = 0;
    virtual int Init() = 0;
    virtual int Process(const FsaiStereoFrame* frame, FsaiDetections* out) = 0;
    virtual void Shutdown() = 0;
};

class IPlannerProvider {
public:
    virtual ~IPlannerProvider() = default;
    virtual const std::string& Name() const = 0;
    virtual int Init() = 0;
    virtual PathMeta BuildPathMeta(const VehicleState& vehicle) = 0;
    virtual void Shutdown() = 0;
};

class IEstimatorProvider {
public:
    virtual ~IEstimatorProvider() = default;
    virtual const std::string& Name() const = 0;
    virtual int Init() = 0;
    virtual FsaiVehicleState Estimate(const VehicleState& truth) = 0;
    virtual void Shutdown() = 0;
};

class ICanProvider {
public:
    virtual ~ICanProvider() = default;
    virtual const std::string& Name() const = 0;
    virtual int Init() = 0;
    virtual int Send(const FsaiControlCmd& cmd) = 0;
    virtual int Poll(FsaiCanMsg* out) = 0;
    virtual void Shutdown() = 0;
};

struct ProviderRegistry;

}  // namespace integration
}  // namespace fsai

