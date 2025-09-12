#include "PathGenerator.hpp"
#include <cmath>
#include <random>

using cdouble = std::complex<double>;

std::vector<double> PathGenerator::computeCornerRadii(double dt, const std::vector<cdouble>& dPdt) const {
    int nPoints = static_cast<int>(dPdt.size());
    std::vector<cdouble> ddPdt(nPoints);
    for (int i = 0; i < nPoints - 1; ++i) {
        ddPdt[i] = (dPdt[i + 1] - dPdt[i]) / dt;
    }
    ddPdt[nPoints - 1] = (dPdt[0] - dPdt[nPoints - 1]) / dt;

    std::vector<double> cornerRadii(nPoints);
    for (int i = 0; i < nPoints; ++i) {
        double num = std::pow(std::abs(dPdt[i]), 3);
        double denom = std::imag(std::conj(dPdt[i]) * ddPdt[i]);
        cornerRadii[i] = (denom == 0.0) ? 0.0 : num / denom;
    }
    return cornerRadii;
}

PathResult PathGenerator::generatePath(int nPoints) const {
    double minCornerRadius = config.minCornerRadius;
    int maxFrequency = config.maxFrequency;
    double amplitude = config.amplitude;

    std::vector<cdouble> z(nPoints);
    std::vector<cdouble> waves(nPoints, cdouble(0,0));
    std::vector<cdouble> dwaves(nPoints, cdouble(0,0));
    std::vector<cdouble> zPow(nPoints);

    for (int t = 0; t < nPoints; ++t) {
        double angle = 2 * M_PI * t / nPoints;
        z[t] = std::exp(cdouble(0, angle));
    }

    std::mt19937 rng(static_cast<unsigned>(config.seed * 1e6));
    std::uniform_real_distribution<double> dist(0.0, 2 * M_PI);

    for (int frequency = 2; frequency <= maxFrequency; ++frequency) {
        double random_phase = dist(rng);
        cdouble phase = std::exp(cdouble(0, random_phase));
        for (int t = 0; t < nPoints; ++t) {
            zPow[t] = std::pow(z[t], frequency);
        }
        double freqPlus = static_cast<double>(frequency + 1);
        double freqMinus = static_cast<double>(frequency - 1);
        for (int t = 0; t < nPoints; ++t) {
            waves[t] += z[t] * ( zPow[t] / (phase * freqPlus) + phase / (zPow[t] * freqMinus) );
            dwaves[t] += ( zPow[t] / phase ) - ( phase / zPow[t] );
        }
    }

    std::vector<cdouble> scaledwaves(nPoints);
    for (int t = 0; t < nPoints; ++t) {
        scaledwaves[t] = waves[t] * amplitude;
    }

    std::vector<cdouble> points(nPoints);
    for (int t = 0; t < nPoints; ++t) {
        points[t] = z[t] + scaledwaves[t];
    }

    std::vector<cdouble> dPdt(nPoints);
    for (int t = 0; t < nPoints; ++t) {
        dPdt[t] = cdouble(0,1) * z[t] * (1.0 + amplitude * dwaves[t]);
    }

    std::vector<cdouble> normalsC(nPoints);
    for (int t = 0; t < nPoints; ++t) {
        double mag = std::abs(dPdt[t]);
        if (mag == 0.0) mag = 1.0;
        normalsC[t] = (cdouble(0,1) * dPdt[t]) / mag;
    }

    double dt = 2 * M_PI / nPoints;
    std::vector<double> cornerRadii = computeCornerRadii(dt, dPdt);

    double minValue = std::numeric_limits<double>::max();
    for (double v : cornerRadii) {
        double absVal = std::abs(v);
        if (absVal < minValue) minValue = absVal;
    }
    double scale = minCornerRadius / minValue;

    std::vector<Eigen::Vector2f> outPoints(nPoints);
    std::vector<Eigen::Vector2f> outNormals(nPoints);
    std::vector<float> outRadii(nPoints);
    for (int t = 0; t < nPoints; ++t) {
        cdouble sp = points[t] * scale;
        outPoints[t] = Eigen::Vector2f(static_cast<float>(sp.real()), static_cast<float>(sp.imag()));
        outNormals[t] = Eigen::Vector2f(static_cast<float>(normalsC[t].real()), static_cast<float>(normalsC[t].imag()));
        outRadii[t] = static_cast<float>(cornerRadii[t] * scale);
    }

    PathResult result;
    result.points = std::move(outPoints);
    result.normals = std::move(outNormals);
    result.cornerRadii = std::move(outRadii);
    return result;
}

