//
// Created by tch on 16.07.18.
//

#ifndef CDFF_NORMALVECTOREXTRACTION_H
#define CDFF_NORMALVECTOREXTRACTION_H

#include <cmath>
#include <iostream>

#include <Types/CPP/Frame.hpp>
#include "ImageFilteringInterface.hpp"

namespace CDFF
{
    namespace DFN {

        class NormalVectorExtraction : public ImageFilteringInterface {
        public:
            void configure() override {}

            void process() override;

        private:

            void ValidateInputs(FrameWrapper::Frame const &frame);
        };

        struct Vec3f {
            float x, y, z;

            Vec3f() = default;

            Vec3f(float s) : x(s), y(s), z(s) {}

            Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}

            inline Vec3f operator*(float s) const {
                return {x * s, y * s, z * s};
            }

            inline Vec3f operator/(float s) const {
                return {x / s, y / s, z / s};
            }

            inline Vec3f operator+(const Vec3f &a) const {
                return {x + a.x, y + a.y, z + a.z};
            }

            inline Vec3f operator-(const Vec3f &a) const {
                return {x - a.x, y - a.y, z - a.z};
            }

            inline bool operator==(const Vec3f &a) const {
                return x == a.x && y == a.y && z == a.z;
            }

            inline bool operator!=(const Vec3f &a) const {
                return !operator==(a);
            }
        };

        inline std::ostream &operator<<(std::ostream &out, const Vec3f &a) {
            out << "[" << a.x << ", " << a.y << ", " << a.z << "]";
            return out;
        }

        inline float norm(const Vec3f &a) {
            return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        }

        inline Vec3f normalize(const Vec3f &a) {
            float a_norm = norm(a);
            if (std::isnan(a.x / a_norm + a.y / a_norm + a.z / a_norm)) {
                return {0.f, 0.f, 0.f};
            }
            return {a.x / a_norm, a.y / a_norm, a.z / a_norm};
        }

        inline float dot(const Vec3f &a, const Vec3f &b) {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        inline Vec3f cross(const Vec3f &a, const Vec3f &b) {
            return {
                    a.z * b.y - a.y * b.z,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x
            };
        }
    }
}

#endif //CDFF_NORMALVECTOREXTRACTION_H
