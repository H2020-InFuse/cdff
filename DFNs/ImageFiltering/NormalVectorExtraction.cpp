//
// Created by tch on 16.07.18.
//

#include "NormalVectorExtraction.hpp"

#include <opencv2/core.hpp>
#include <Errors/Assert.hpp>
#include <Types/C/Frame.h>
#include "Validators.hpp"

namespace CDFF {
    namespace DFN{
        namespace ImageFiltering {

            /** To compute the normals of the image we extract the 8-connected
             *  neighbourhood of the pixel. The points are labelled as:
             *
             *       +------+------+------+   tl = top left         bl = bottom left
             *       |  tl  |  tc  |  tr  |   tc = top center       bc = bottom center
             *       +------+------+------+   tr = top right        br = bottom right
             *       |  cl  |  cc  |  cr  |
             *       +------+------+------+   cl = center left
             *       |  bl  |  bc  |  br  |   cc = center center
             *       +------+------+------+   cr = center left
             *
             *  These points are then used to compute the normals of the triangles
             *  surrounding the center point.
             *
             *                 (tc)
             *  (tl) +----------+----------+ (tr)   The vector normal to the surface of
             *       |  *   tl  |  tr   *  |        triangle _tl_ is:
             *       |     *    |    *     |
             *       |  lt    * | *    rt  |          tl_n = (tc - cc) x (tl - cc)
             *  (cl) +----------+----------+ (cr)
             *       |  lb    * | *    rb  |        where _x_ is the cross product.
             *       |     *    |    *     |
             *       |  *   bl  |  br   *  |
             *  (bl) +----------+----------+ (br)
             *                 (bc)
             */
            void NormalVectorExtraction::process() {
                ValidateInputs(inImage);
                const auto *inPixels = reinterpret_cast<Vec3f *>(inImage.data.data.arr);

                FrameWrapper::FrameSharedPtr normals = FrameWrapper::NewSharedFrame();
                FrameWrapper::Copy(inImage, *normals);
                FrameWrapper::ClearData(*normals, /* overwrite = */ true);

                // Reinterpret the image data as a vector of Vec3f. This will make it easier to perform
                // all the computations without spending too much time reconstructing vectors and
                // computing offsets.
                auto *normalPixels = reinterpret_cast<Vec3f *>(normals->data.data.arr);

                const size_t width = inImage.data.cols;
                for (size_t row = 1; row < inImage.data.rows - 1; ++row) {
                    for (size_t col = 1; col < inImage.data.cols - 1; ++col) {
                        // Extract all of the pixels in the 8-connected neighbourhood of the center pixel (cc)
                        Vec3f tl = inPixels[(row - 1) * width + (col - 1)];
                        Vec3f tc = inPixels[(row - 1) * width + (col)];
                        Vec3f tr = inPixels[(row - 1) * width + (col + 1)];
                        Vec3f cl = inPixels[(row) * width + (col - 1)];
                        Vec3f cc = inPixels[(row) * width + (col)];
                        Vec3f cr = inPixels[(row) * width + (col + 1)];
                        Vec3f bl = inPixels[(row + 1) * width + (col - 1)];
                        Vec3f bc = inPixels[(row + 1) * width + (col)];
                        Vec3f br = inPixels[(row + 1) * width + (col + 1)];

                        // Compute the normals to the vectors linking the center pixel to each one of
                        // the neighbours and normalize them
                        Vec3f tl_n = normalize(cross(tc - cc, tl - cc));
                        Vec3f tr_n = normalize(cross(tr - cc, tc - cc));
                        Vec3f rt_n = normalize(cross(cr - cc, tr - cc));
                        Vec3f rb_n = normalize(cross(br - cc, cr - cc));
                        Vec3f br_n = normalize(cross(bc - cc, br - cc));
                        Vec3f bl_n = normalize(cross(bl - cc, bc - cc));
                        Vec3f lb_n = normalize(cross(cl - cc, bl - cc));
                        Vec3f lt_n = normalize(cross(tl - cc, cl - cc));

                        // Compute the mean of all the triangle normals
                        Vec3f normal_vec = normalize(
                                (tl_n + tr_n + rt_n + rb_n + br_n + bl_n + lb_n + lt_n) / 8.f);

                        // Set the result
                        normalPixels[row * width + col] = normal_vec;
                    }
                }

                outImage = *normals;
            }

            void NormalVectorExtraction::ValidateInputs(const FrameWrapper::Frame &frame) {
                Validators::Frame::NotEmpty(frame);
                Validators::Frame::HasDepthOf(frame, Array3DWrapper::Array3DDepth::asn1Sccdepth_32S);
                Validators::Frame::HasFormatIn(frame, {FrameWrapper::MODE_XYZ});
                Validators::Frame::HasStatus(frame, FrameWrapper::STATUS_VALID);
            }
        } // End namespace ImageFiltering
    } // End namespace DFN
} // End namespace CDFF