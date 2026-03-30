/*************************************************************************
 *
 * ADOBE CONFIDENTIAL
 * ___________________
 *
 *  Copyright 2025 Adobe Systems Incorporated
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of Adobe Systems Incorporated and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Adobe Systems Incorporated and its
 * suppliers and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Adobe Systems Incorporated.
 **************************************************************************/

#pragma once

/** \file
* utility functions for bezier skinning.
*/

#include <utility>
#include <vector>

#include "optimtools/all.h"

#include "sketch-foundation/geom/Vector.h"

#ifdef WIN_ENV
__pragma(warning(push))
__pragma(warning(disable:4267))
__pragma(warning(disable:4244))
__pragma(warning(disable:4018))
#endif

namespace extensions
{
    namespace puppet
    {
        // a triangle index + uvw coordinate within a triangle. This representation is independent of the embedding.
        struct MeshLocation
        {
            MeshLocation() { tri = -1; }
            MeshLocation(int _tri, const geom::Vector<3,double> &_uvw) { tri = _tri; uvw = _uvw; }
            
            // get the embedded position, given the points and triangles of the mesh.
            geom::Vector2f getPosition(const optimtools::mesh_traits<double, 2>::PointVector &points,
                                       const optimtools::mesh_traits<double, 2>::TriangleVector &triangles) const
            {
                const auto &t = triangles[tri];
                const auto &p0 = points[t[0]];
                const auto &p1 = points[t[1]];
                const auto &p2 = points[t[2]];
                
                return geom::Vector2f(p0.x(), p0.y()) * uvw(0) +
                       geom::Vector2f(p1.x(), p1.y()) * uvw(1) +
                       geom::Vector2f(p2.x(), p2.y()) * uvw(2);
            }
            
            // index into the mesh's triangle list
            int64_t tri;
            
            int64_t puppet_index = -1;
            
            // UVW coordinate within this triangle
            geom::Vector<3,double> uvw;
        };
    
        // LSQSystem facilitates the construction and factorization of a least-squares system.
        // LSQSystem also stores a mutable b-vector, and will solve using this vector.
        class LSQSystem
        {
        public:
            LSQSystem() {
                m_constraintCount = 0;
                m_xCount = -1;
            }
            
            void reset(int xCount) {
                m_constraintCount = 0;
                m_xCount = xCount;
                m_AMatrixTriplets.clear();
            }
            
            // add a new row to the LSQ system
            void addConstraint(const std::vector< std::pair<int, double> > &terms, double weight)
            {
                for (const auto &t : terms)
                {
                    if (t.second != 0.0)
                        m_AMatrixTriplets.push_back(Triplet(m_constraintCount, t.first, t.second * weight));
                }
                m_constraintCount++;
            }
            
            // Computes A matrix with all added constraints constraints. Factorization of ATA, and
            // sets up b for repetitive solving
            void finalizeProblem()
            {
                //		cout << "Finalizing ATA..." << endl;
                m_A.resize(m_constraintCount, m_xCount);
                m_A.setFromTriplets(m_AMatrixTriplets.begin(), m_AMatrixTriplets.end());
                m_AT = m_A.transpose();
                m_ATA = m_AT * m_A;
                
                m_ATAFactorization.compute(m_ATA);
                
                m_b = Eigen::VectorXd(m_constraintCount);
                // TODO: verify Eigen zeroes everything
                for (int i = 0; i < m_constraintCount; i++)
                    m_b(i) = 0.0;
                //		cout << "done" << endl;
            }
            
            // solves for a new x given the current b
            Eigen::VectorXd solve() const
            {
                Eigen::VectorXd ATb = m_AT * m_b;
                return m_ATAFactorization.solve(ATb);
            }
            
            // mutable accessor to the current b vector. This makes it easy to only update a part of b.
            Eigen::VectorXd& b()
            {
                return m_b;
            }
            
        private:
            // Eigen reference: https://eigen.tuxfamily.org/dox-devel/group__TutorialSparse.html
            typedef Eigen::SparseMatrix<double> SparseMatrix;
            typedef Eigen::Triplet<double> Triplet;
            
            // the number of constraints that have been added so far
            int m_constraintCount;
            
            // the number of unknowns
            int m_xCount;
            
            // the built-up set of triplets
            std::vector< Triplet > m_AMatrixTriplets;
            
            //A in A^T A x = A^T b
            SparseMatrix m_A;
            SparseMatrix m_AT;
            SparseMatrix m_ATA;
            Eigen::SimplicialCholesky<SparseMatrix> m_ATAFactorization;
            Eigen::VectorXd m_b;
        };
        
        /*MeshLocation findMeshCoordinate(const vec2f &pt, const mesh_traits<double, 2>::PointVector& points, const mesh_traits<double, 2>::TriangleVector& triangles)
         {
         for (int triIndex = 0; triIndex < triangles.size(); triIndex++)
         {
         const auto &optimTri = triangles[triIndex];
         const auto &p0 = points[optimTri[0]];
         const auto &p1 = points[optimTri[1]];
         const auto &p2 = points[optimTri[2]];
         
         ml::Triangle2f tri(vec2f(p0.x(), p0.y()), vec2f(p1.x(), p1.y()), vec2f(p2.x(), p2.y()));
         
         if (tri.contains(pt))
         {
         const vec3f uvw = tri.getUVW(pt);
         return MeshLocation(triIndex, uvw);
         }
         }
         
         //cout << "No triangle found" << endl;
         //return MeshLocation();
         
         // if no triangle is found, search again and pick the closest
         MeshLocation bestLocation;
         float bestDistSq = std::numeric_limits<float>::max();
         for (int triIndex = 0; triIndex < triangles.size(); triIndex++)
         {
         const auto &optimTri = triangles[triIndex];
         const auto &p0 = points[optimTri[0]];
         const auto &p1 = points[optimTri[1]];
         const auto &p2 = points[optimTri[2]];
         
         ml::Triangle2f tri(vec2f(p0.x(), p0.y()), vec2f(p1.x(), p1.y()), vec2f(p2.x(), p2.y()));
         
         const float distSq = tri.distSq(pt);
         if(distSq < bestDistSq)
         {
         bestDistSq = distSq;
         const vec3f uvw = tri.getUVW(pt);
         bestLocation = MeshLocation(triIndex, uvw);
         }
         }
         
         return bestLocation;
         }
         
         struct SVGProcessor
         {
         static void SVGToPNG(const string &svgFilename, const string &pngFilename)
         {
         if (util::fileExists(pngFilename))
         {
         cout << pngFilename << " already exists" << endl;
         return;
         }
         
         NSVGimage* image = nsvgParseFromFile(svgFilename.c_str(), "px", 300);
         
         // Create rasterizer (can be used to render multiple images).
         struct NSVGrasterizer* rast = nsvgCreateRasterizer();
         
         int width = int(image->width + 1.0);
         int height = int(image->height + 1.0);
         ColorImageR8G8B8A8 bitmap(width, height);
         
         // Rasterize
         nsvgRasterize(rast, image, 0, 0, 1, (unsigned char *)bitmap.getData(), width, height, width * 4);
         
         // LodePNG will save as a palette if it uses fewer than 256 colors and some code cannot read palette PNGs, so scamble the image's color since all we care about is the mask.
         for (const auto &p : bitmap)
         {
         p.value.r = 80 + rand() % 16;
         p.value.g = 80 + rand() % 16;
         p.value.b = 220 + rand() % 16;
         }
         
         LodePNG::save(bitmap, pngFilename, true);
         }
         };*/
#if 0
        struct TransformerUtil
        {
            static void runTests()
            {
                vector<vec2f> A, B;
                A.push_back(vec2f(-1.0f, -1.0f));
                A.push_back(vec2f(1.0f, -1.0f));
                A.push_back(vec2f(1.0f, 1.0f));
                A.push_back(vec2f(-1.0f, 1.0f));
                B = A;
                
                //		mat4f translation = mat4f::translation(100.0f, 10.0f, 0.0f);
                //		mat4f scale = mat4f::scale(2.0f);
                //		mat4f rotation = mat4f::rotationZ(10.0f);
                /*for (const auto &b : B)
                 {
                 b = (translation * rotation * scale * vec3f(b, 0.0f)).getVec2();
                 //b = (scale * vec3f(b, 0.0f)).getVec2();
                 //b = (translation * scale * vec3f(b, 0.0f)).getVec2();
                 }*/
                
                mat4f aToB = getFullTransform(A, B);
                
                for (int i = 0; i < (int)A.size(); i++)
                {
                    const vec2f bOut = (aToB * vec3f(A[i], 0.0f)).getVec2();
                    cout << B[i].toString(',') << " -> " << bOut.toString(',') << endl;
                }
            }
            
            // computes the optimal rigid/reflection+scale transform from points set A to B, assuming 1-to-1 correspondences between the points
            static mat4f getFullTransform(const vector<vec2f> &ABase, const vector<vec2f> &BBase)
            {
                vector<vec2f> A = ABase;
                vector<vec2f> B = BBase;
                constexpr auto FLOAT_MAX = std::numeric_limits<float>::max();
                vec2f aMin(FLOAT_MAX,FLOAT_MAX) , bMin(FLOAT_MAX,FLOAT_MAX);
                
                for (const vec2f &a : A) {aMin.x =std::min<float>(a.x, aMin.x); aMin.y =std::min<float>(a.y, aMin.y);}
                for (const vec2f &b : B) {bMin.x =std::min<float>(b.x, bMin.x); bMin.y =std::min<float>(b.y, bMin.y);}
                for (vec2f &a : A) a -= aMin;
                for (vec2f &b : B) b -= bMin;
                
                vec2f aCentroid, bCentroid;
                for (const auto &a : A) aCentroid += a;
                for (const auto &b : B) bCentroid += b;
                
                aCentroid /= (float)A.size();
                bCentroid /= (float)B.size();
                
                for (auto &a : A) a -= aCentroid;
                for (auto &b : B) b -= bCentroid;
                
                aCentroid += aMin;
                bCentroid += bMin;
                
                
                float scaleA = std::numeric_limits<float>::epsilon(), scaleB = std::numeric_limits<float>::epsilon();
                for (auto &a : A) scaleA += a.lengthSq();
                for (auto &b : B) scaleB += b.lengthSq();
                scaleA = 1.0f / sqrt(scaleA / (float)A.size());
                scaleB = 1.0f / sqrt(scaleB / (float)B.size());
                
                for (auto &a : A) a *= scaleA;
                for (auto &b : B) b *= scaleB;
                
                const mat4f rotation = kabsch(A, B);
                
                const mat4f result = mat4f::translation(vec3f(bCentroid, 0.0f)) * mat4f::scale(scaleA / scaleB) * rotation * mat4f::translation(vec3f(-aCentroid, 0.0f));
                
                return result;
            }
            
            static Eigen::MatrixXd computeCovariance(const vector<vec2f> &A, const vector<vec2f> &B)
            {
                const int ACount = (int)A.size();
                const int BCount = (int)B.size();
                Eigen::MatrixXd AMatrix(ACount, 2);
                Eigen::MatrixXd BMatrix(BCount, 2);
                
                for (int a = 0; a < ACount; a++)
                    for (int d = 0; d < 2; d++)
                        AMatrix(a, d) = A[a][d];
                
                for (int b = 0; b < BCount; b++)
                    for (int d = 0; d < 2; d++)
                        BMatrix(b, d) = B[b][d];
                
                AMatrix.transposeInPlace();
                return AMatrix * BMatrix;
            }
            
            // computes the optimal rotation-only matrix from A to B, assuming 1-to-1 correspondences between the points
            // TODO: replace this with a call to optimtools
            static mat4f kabsch(const vector<vec2f> &A, const vector<vec2f> &B)
            {
                Eigen::MatrixXd covariance = computeCovariance(A, B);
                
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
                double determinant = (svd.matrixV() * svd.matrixU().transpose()).determinant();
                Eigen::MatrixXd D = Eigen::MatrixXd::Identity(2, 2);
                if (determinant < 0) D(1, 1) = -1;
                Eigen::MatrixXd R = svd.matrixU() * D * svd.matrixV().transpose();
                
                mat4d result = mat4f::identity();
                for (int x = 0; x < 2; x++)
                    for (int y = 0; y < 2; y++)
                        result(y, x) = R(x, y);
                return result;
            }
        };
#endif
    }
}

#ifdef WIN_ENV
__pragma(warning(pop))
#endif
