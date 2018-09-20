//
// Created by tch on 16.07.18.
//

#ifndef CDFF_NORMALVECTOREXTRACTION_H
#define CDFF_NORMALVECTOREXTRACTION_H

#include <cmath>
#include <iostream>

#include <Types/CPP/Frame.hpp>
#include "ImageFilteringInterface.hpp"

namespace CDFF {
    namespace DFN {

        namespace ImageFiltering {
            /**
             * The NormalVector Extraction DFN computes the normals from a depth image.
             *
             * There are different ways to compute normals. Some methods consider the 4-connected
             * neighbourhood of the point, other look for wider trends.
             *
             * This DFN considers the 8 connected neighbourhood, creates triangles comprised of two
             * consecutive neighbour pixels and the center pixel. The normals of all these triangles
             * are computed, normalised and averaged to produce the final normal vector.
             * This _should_ provide a set of smoother normal vectors.
             *
             * For more details on the method consult the documentation for the
             * NormalVectorExtraction::process method.
             */
            class NormalVectorExtraction : public ImageFilteringInterface {
            public:
                void configure() override {}

                void process() override;

            private:

                void ValidateInputs(FrameWrapper::Frame const &frame);
            };

            /**
             * Simple Vector3 class only supporting the operation we need.
             *
             * In a perfect world this type would be sowmhere in the Common/ tree but since it is
             * relatively specialised (no support for doubles or other sizes of vectors) and other
             * libraries provide faster matrix/vector operations this might not make sense. The goal
             * of this class was to have a structure with a known and simple layout we could use to
             * reinterpret the entire image data without having to manually cast each element. For
             * this reason it has no v-table. It should be a POD type for the casting to work as
             * inteneded.
             *
             * The benefit of this approach is that it gives us one non-trivial DFN with no
             * dependencies on OpenCV and PCL!
             */
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

            /// Print vector `a` to the ostream `out`
            inline std::ostream &operator<<(std::ostream &out, const Vec3f &a) {
                out << "[" << a.x << ", " << a.y << ", " << a.z << "]";
                return out;
            }

            /// Compute the norm of vector `a`
            inline float norm(const Vec3f &a) {
                return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
            }

            /// Normalize vector `a`. If any component is NaN then a 0 vector is returned.
            inline Vec3f normalize(const Vec3f &a) {
                float a_norm = norm(a);
                if (std::isnan(a.x / a_norm + a.y / a_norm + a.z / a_norm)) {
                    return {0.f, 0.f, 0.f};
                }
                return {a.x / a_norm, a.y / a_norm, a.z / a_norm};
            }

            /// Compute the dot product of vectors `a` and `b`
            inline float dot(const Vec3f &a, const Vec3f &b) {
                return a.x * b.x + a.y * b.y + a.z * b.z;
            }

            /// Compute the cross product of vectors `a` and `b`
            inline Vec3f cross(const Vec3f &a, const Vec3f &b) {
                return {
                        a.z * b.y - a.y * b.z,
                        a.z * b.x - a.x * b.z,
                        a.x * b.y - a.y * b.x
                };
            }
        } // End namespace ImageFiltering
    } // End namespace DFN
} // End namespace CDFF
#endif //CDFF_NORMALVECTOREXTRACTION_H
