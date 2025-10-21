#include "sim/integration/fake_providers.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>

#include "fsai_clock.h"

namespace fsai {
namespace integration {
namespace {

class FakeVisionProvider : public IVisionProvider {
public:
    FakeVisionProvider(const PathTruth& truth, const FakeVisionOptions& options)
        : truth_(truth), options_(options), rng_(1234u),
          noise_dist_(0.0f, options.position_noise_std),
          dropout_dist_(0.0f, 1.0f), conf_dist_(options.min_conf, options.max_conf) {}

    const std::string& Name() const override { return name_; }

    int Init() override { return 0; }

    int Process(const FsaiStereoFrame* /*frame*/, FsaiDetections* out) override {
        out->t_ns = fsai_clock_now();
        out->n = 0;

        auto emit = [&](const Vector3& cone, FsaiConeSide side) {
            if (out->n >= 512) {
                return;
            }
            if (dropout_dist_(rng_) < options_.dropout_probability) {
                return;
            }
            FsaiConeDet& det = out->dets[out->n++];
            det.p_B[0] = cone.x + noise_dist_(rng_);
            det.p_B[1] = cone.y;
            det.p_B[2] = cone.z + noise_dist_(rng_);
            det.side = side;
            det.conf = conf_dist_(rng_);
            std::fill(std::begin(det.cov), std::end(det.cov), 0.0f);
        };

        for (const Transform& t : truth_.track.leftCones) {
            emit(t.position, FSAI_CONE_LEFT);
        }
        for (const Transform& t : truth_.track.rightCones) {
            emit(t.position, FSAI_CONE_RIGHT);
        }

        return 1;
    }

    void Shutdown() override {}

private:
    const std::string name_ = "fake_from_sim";
    const PathTruth& truth_;
    FakeVisionOptions options_;
    std::mt19937 rng_;
    std::normal_distribution<float> noise_dist_;
    std::uniform_real_distribution<float> dropout_dist_;
    std::uniform_real_distribution<float> conf_dist_;
};

class FakePlannerProvider : public IPlannerProvider {
public:
    FakePlannerProvider(const PathTruth& truth, const FakePlannerOptions& options)
        : truth_(truth), options_(options) {}

    const std::string& Name() const override { return name_; }

    int Init() override { return 0; }

    PathMeta BuildPathMeta(const VehicleState& vehicle) override {
        PathMeta meta;
        meta.centerline = &truth_.centerline;
        meta.arc_length = &truth_.centerline_arc_length;
        meta.sample_spacing_m = options_.sample_spacing_m;
        meta.horizon_m = options_.horizon_m;
        meta.vehicle_position.x = static_cast<float>(vehicle.position.x());
        meta.vehicle_position.y = 0.0f;
        meta.vehicle_position.z = static_cast<float>(vehicle.position.y());
        meta.vehicle_yaw = static_cast<float>(vehicle.yaw);
        return meta;
    }

    void Shutdown() override {}

private:
    const std::string name_ = "fake_track";
    const PathTruth& truth_;
    FakePlannerOptions options_;
};

class FakeEstimatorProvider : public IEstimatorProvider {
public:
    const std::string& Name() const override { return name_; }

    int Init() override { return 0; }

    FsaiVehicleState Estimate(const VehicleState& truth) override {
        FsaiVehicleState out{};
        out.t_ns = truth.timestampNs;
        out.x = static_cast<float>(truth.position.x());
        out.y = static_cast<float>(truth.position.y());
        out.yaw = static_cast<float>(truth.yaw);
        out.vx = static_cast<float>(truth.velocity.x());
        out.vy = static_cast<float>(truth.velocity.y());
        out.yaw_rate = static_cast<float>(truth.rotation.z());
        out.ax = static_cast<float>(truth.acceleration.x());
        out.ay = static_cast<float>(truth.acceleration.y());
        out.steer_rad = 0.0f;
        return out;
    }

    void Shutdown() override {}

private:
    const std::string name_ = "fake_truth";
};

class FakeCanProvider : public ICanProvider {
public:
    const std::string& Name() const override { return name_; }

    int Init() override {
        last_send_ns_ = 0;
        last_heartbeat_ns_ = fsai_clock_now();
        return 0;
    }

    int Send(const FsaiControlCmd& cmd) override {
        last_send_ns_ = cmd.t_ns;
        last_cmd_ = cmd;
        return 1;
    }

    int Poll(FsaiCanMsg* out) override {
        const uint64_t now = fsai_clock_now();
        if (now - last_heartbeat_ns_ >= heartbeat_period_ns_) {
            last_heartbeat_ns_ = now;
            out->id = 0x100u;
            out->dlc = 2;
            out->data[0] = 1;  // Ready/Go flag
            out->data[1] = 1;  // RES safe flag
            out->t_ns = now;
            return 1;
        }
        return 0;
    }

    void Shutdown() override {}

    const FsaiControlCmd& LastCommand() const { return last_cmd_; }
    uint64_t LastSendTime() const { return last_send_ns_; }

private:
    const std::string name_ = "fake";
    uint64_t last_send_ns_{0};
    uint64_t last_heartbeat_ns_{0};
    FsaiControlCmd last_cmd_{};
    static constexpr uint64_t heartbeat_period_ns_ = 10'000'000ull;  // 10 ms
};

}  // namespace

std::unique_ptr<IVisionProvider> MakeFakeVisionProvider(const PathTruth& truth, const FakeVisionOptions& options) {
    return std::make_unique<FakeVisionProvider>(truth, options);
}

std::unique_ptr<IPlannerProvider> MakeFakePlannerProvider(const PathTruth& truth, const FakePlannerOptions& options) {
    return std::make_unique<FakePlannerProvider>(truth, options);
}

std::unique_ptr<IEstimatorProvider> MakeFakeEstimatorProvider() {
    return std::make_unique<FakeEstimatorProvider>();
}

std::unique_ptr<ICanProvider> MakeFakeCanProvider() {
    return std::make_unique<FakeCanProvider>();
}

}  // namespace integration
}  // namespace fsai

