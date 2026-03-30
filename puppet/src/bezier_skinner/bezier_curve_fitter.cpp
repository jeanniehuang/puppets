/*************************************************************************
* ADOBE CONFIDENTIAL
* ___________________
*
*  Copyright 2025 Adobe
*  All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains
* the property of Adobe and its suppliers, if any. The intellectual
* and technical concepts contained herein are proprietary to Adobe
* and its suppliers and are protected by all applicable intellectual
* property laws, including trade secret and copyright laws.
* Dissemination of this information or reproduction of this material
* is strictly forbidden unless prior written permission is obtained
* from Adobe.
**************************************************************************/

#include "bezier_curve_fitter.h"

namespace extensions
{
    namespace puppet
    {
        /*
            *  B0, B1, B2, B3 :
            *  Bezier multipliers
            */
        static inline double        B0(double u) { double tmp = 1.0 - u; return     tmp * tmp * tmp; }
        static inline double        B1(double u) { double tmp = 1.0 - u; return 3 * u   * tmp * tmp; }
        static inline double        B2(double u) { double tmp = 1.0 - u; return 3 * u   * u   * tmp; }
        static inline double        B3(double u) {                         return     u   * u   * u;   }
        /*
        *  EvalBezier :
        *      Evaluate a Bezier curve at a particular parameter value
        *
        */
        #define MAX_DEGREE 5
        static void EvalBezier(const std::vector<geom::Vector2f> &V, size_t degree, double t, double result[2])
        {
            if (degree+1 > MAX_DEGREE) {
                result[0] = V[0](0);
                result[1] = V[0](1);
                return;
            }

            double Vtemp[MAX_DEGREE][2]; // Local copy of control points

            /* Copy array   */
            for (auto i = 0; i <= degree; i++)
            {
                auto &vi = V[i];
                Vtemp[i][0] = vi.x;
                Vtemp[i][1] = vi.y;
            }

            /* Triangle computation */
            for (auto i = 1; i <= degree; i++)
            {
                for (auto j = 0; j <= degree - i; j++)
                {
                    Vtemp[j][0] = (1.0 - t) * Vtemp[j][0] + t * Vtemp[j + 1][0];
                    Vtemp[j][1] = (1.0 - t) * Vtemp[j][1] + t * Vtemp[j + 1][1];
                }
            }
            
            result[0] = Vtemp[0][0];
            result[1] = Vtemp[0][1];// Point on curve at parameter t
        }

        static void EvalBezierFast(const geom::Vector2f p[4], double t, double result[2])
        {
            static size_t n = 3;
            double u, bc, tn, tmpX, tmpY;
            u = 1.0 - t;
            bc = 1;
            tn = 1;

            tmpX = p[0].x * u;
            tmpY = p[0].y * u;
            tn = tn * t;
            bc = bc * (n - 1 + 1) / 1;
            tmpX = (tmpX + tn * bc * p[1].x) * u;
            tmpY = (tmpY + tn * bc * p[1].y) * u;
            tn = tn * t;
            bc = bc * (n - 2 + 1) / 2;
            tmpX = (tmpX + tn * bc * p[2].x) * u;
            tmpY = (tmpY + tn * bc * p[2].y) * u;
             
            result[0] = tmpX + tn * t * p[3].x;
            result[1] = tmpY + tn * t * p[3].y;
        }
        /*
            * ComputeLeftTangent, ComputeRightTangent, ComputeCenterTangent :
            *Approximate unit tangents at endpoints and "center" of digitized curve
            */
        static geom::Vector2f ComputeLeftTangent(const geom::Vector2f d[], size_t end)
        {
            geom::Vector2f tHat1;
            size_t use = 1;
            tHat1 = d[end + use] - d[end];
            
            if (std::abs(tHat1.x) < 0.001 && std::abs(tHat1.y) < 0.001)
                return geom::Vector2f(0,0);

            return tHat1.unit();
        }
        static geom::Vector2f ComputeRightTangent(const geom::Vector2f d[], size_t end)
        {
            geom::Vector2f tHat2;
            auto use = 1;
            tHat2 = d[end - use] - d[end];
            
            if (std::abs(tHat2.x) < 0.001 && std::abs(tHat2.y) < 0.001)
                return geom::Vector2f(0,0);

            return tHat2.unit();
        }
        static geom::Vector2f ComputeCenterTangent(const geom::Vector2f d[], size_t center)
        {
            geom::Vector2f V1, V2, tHatCenter;
            if (center == 0)
            {
                return ComputeLeftTangent(d, center);
            }
            V1 = -ComputeLeftTangent(d, center); // d[center - 1] - d[center];
            V2 =  ComputeRightTangent(d, center); // d[center] - d[center + 1];
            tHatCenter.x = (V1.x + V2.x) / 2.0;
            tHatCenter.y = (V1.y + V2.y) / 2.0;
            
            if (std::abs(tHatCenter.x) < 0.001 && std::abs(tHatCenter.y) < 0.001)
                return geom::Vector2f(0,0);

            return tHatCenter.unit();
        }
        static void GenerateBezier(const geom::Vector2f d[], size_t first, size_t last,
                                   const std::vector<double> &uPrime,
                                   const geom::Vector2f &tHat1, const geom::Vector2f &tHat2,
                                   geom::Vector2f result[4] /* must be prealloacted to size 4 */)
        {
            const size_t MAXPTS = 1000;
            size_t nPts = last - first + 1; // Number of pts in sub-curve
            double _Ax[MAXPTS*2], *Ax = _Ax;// Precomputed rhs for eqn  //std::vector<Vector2D> A(nPts * 2);
            double _Ay[MAXPTS*2], *Ay = _Ay;// Precomputed rhs for eqn  //std::vector<Vector2D> A(nPts * 2);
            if (nPts > MAXPTS) {
                Ax = new double[nPts*2];
                Ay = new double[nPts*2];
            }

            /* Compute the A's  */
            for (auto i = 0; i < nPts; i++)
            {
                auto uprime = uPrime[i];
                auto b1 = B1(uprime);
                auto b2 = B2(uprime);
                Ax[i]        = tHat1(0) * b1;
                Ay[i]        = tHat1(1) * b1;
                Ax[i+1*nPts] = tHat2(0) * b2;
                Ay[i+1*nPts] = tHat2(1) * b2;
            }

            /* Create the C and X matrices  */
            double C[2][2] = { {0,0}, {0,0} };
            auto &df = d[first];
            auto &dl = d[last];
            
            double X[2] = {0,0};    // Matrix X
            for (auto i = 0; i < nPts; i++)
            {
                C[0][0] += Ax[i]*Ax[i]      + Ay[i]*Ay[i];     //A[i+0*nPts].Dot(A[i+0*nPts]);
                C[0][1] += Ax[i]*Ax[i+nPts] + Ay[i]*Ay[i+nPts];//A[i+0*nPts].Dot(A[i+1*nPts]);
                C[1][0] = C[0][1];
                C[1][1] += Ax[i+nPts]*Ax[i+nPts] + Ay[i+nPts]*Ay[i+nPts];// A[i+1*nPts].Dot(A[i+1*nPts]);
                auto uprime = uPrime[i];
                auto b0plb1 = B0(uprime) + B1(uprime);
                auto b2plb3 = B2(uprime) + B3(uprime);
                auto df1 = d[first+i];
                auto tmpX = df1.x - (df.x * b0plb1 + (dl.x * b2plb3));
                auto tmpY = df1.y - (df.y * b0plb1 + (dl.y * b2plb3));

                X[0] += Ax[i]      * tmpX + Ay[i]      * tmpY; // A[i+0*nPts].Dot(tmp)
                X[1] += Ax[i+nPts] * tmpX + Ay[i+nPts] * tmpY; //A[i+1*nPts].Dot(tmp)
            }

            /* Compute the determinants of C and X  */
            auto det_C0_C1 = C[0][0] * C[1][1] - C[1][0] * C[0][1];
            auto det_C0_X  = C[0][0] * X[1]    - C[0][1] * X[0];
            auto det_X_C1  = X[0]    * C[1][1] - X[1]    * C[0][1];

            /* Finally, derive alpha values    */
            if (det_C0_C1 == 0.0) {
                det_C0_C1 = (C[0][0] * C[1][1]) * 10e-12;
            }
            auto alpha_l = (det_C0_C1 == 0) ? 0.0 : det_X_C1 / det_C0_C1;
            auto alpha_r = (det_C0_C1 == 0) ? 0.0 : det_C0_X / det_C0_C1;

            /* If alpha negative, use the Wu/Barsky heuristic (see text) */
            /* (if alpha is 0, you get coincident control points that lead to
             * divide by zero in any subsequent NewtonRaphsonRootFind() call. */
            double segLength = (df - dl).norm2();
            double epsilon   = 1.0e-6 * segLength;
            if (alpha_l < epsilon || alpha_r < epsilon)
            {
                /* fall back on standard (probably inaccurate) formula, and subdivide further if needed. */
                alpha_l = alpha_r = segLength / 3.0;
            }

            /*  First and last control points of the Bezier curve are */
            /*  positioned exactly at the first and last data points */
            /*  Control points 1 and 2 are positioned an alpha distance out */
            /*  on the tangent vectors, left and right, respectively */
            result[0] = df;// RETURN bezier curve ctl pts
            result[3] = dl;
            result[1] = (tHat1 * alpha_l) + df;
            result[2] = (tHat2 * alpha_r) + dl;

            if (nPts > MAXPTS)
            {
                delete [] Ax;
                delete [] Ay;
            }
        }
        /*
         *  NewtonRaphsonRootFind :
         *  Use Newton-Raphson iteration to find better root.
         */
        static double NewtonRaphsonRootFind(const geom::Vector2f Q[4], const geom::Vector2f &P, double u)
        {
            std::vector<geom::Vector2f> Q1(3), Q2(2); //  Q' and Q''
            double Q_u[2], Q1_u[2], Q2_u[2]; //u evaluated at Q, Q', & Q''

            /* Compute Q(u) */
            double uPrime; //  Improved u
            EvalBezierFast(Q, u, Q_u);

            /* Generate control vertices for Q' */
            for (auto i = 0; i <= 2; i++)
            {
                Q1[i].x = (Q[i + 1].x - Q[i].x) * 3.0;
                Q1[i].y = (Q[i + 1].y - Q[i].y) * 3.0;
            }

            /* Generate control vertices for Q'' */
            for (auto i = 0; i <= 1; i++)
            {
                Q2[i].x = (Q1[i + 1].x - Q1[i].x) * 2.0;
                Q2[i].y = (Q1[i + 1].y - Q1[i].y) * 2.0;
            }

            /* Compute Q'(u) and Q''(u) */
            EvalBezier(Q1, 2, u, Q1_u);
            EvalBezier(Q2, 1, u, Q2_u);

            /* Compute f(u)/f'(u) */
            auto numerator   = (Q_u[0] - P.x) * (Q1_u[0]) + (Q_u[1] - P.y) * (Q1_u[1]);
            auto denominator = (Q1_u[0]) * (Q1_u[0]) + (Q1_u[1]) * (Q1_u[1]) + (Q_u[0] - P.x) * (Q2_u[0]) + (Q_u[1] - P.y) * (Q2_u[1]);
            if (denominator == 0.0f)
                 uPrime = u;
            else uPrime = u - (numerator / denominator);/* u = u - f(u)/f'(u) */

            return uPrime;
        }
        static void FitCubic(const geom::Vector2f d[], size_t first, size_t last,
                             const geom::Vector2f &tHat1, const geom::Vector2f &tHat2,
                             double error, std::vector<geom::Vector2f> &result)
        {
            geom::Vector2f       bezCurve[4];            //  Control points of fitted Bezier curve
            size_t               splitPoint2D;           //  Point2D to split point set at
            int                  maxIterations = 2;      //  Max times to try iterating
            
            auto nPts = last - first + 1;        //  Number of points in subset

            /*  Use heuristic if region only has two points in it */
            if (nPts == 2)
            {
                double dist = (d[first] - d[last]).norm2() / 3.0;

                bezCurve[0] = d[first];
                bezCurve[3] = d[last];
                bezCurve[1] = (tHat1 * dist) + bezCurve[0];
                bezCurve[2] = (tHat2 * dist) + bezCurve[3];

                result.push_back(bezCurve[1]);
                result.push_back(bezCurve[2]);
                result.push_back(bezCurve[3]);
                return;
            }

            /*  Parameterize points, and attempt to fit curve */
            auto u = BezierRep::ChordLengthParameterize(d, first, last);
            GenerateBezier(d, first, last, u, tHat1, tHat2, bezCurve);

            /*  Find max deviation of points to fitted curve */
            auto maxError = BezierRep::ComputeMaxError(d, first, last, bezCurve, u, &splitPoint2D);  //  Maximum fitting error
            result.push_back(bezCurve[1]);
            result.push_back(bezCurve[2]);
            result.push_back(bezCurve[3]);

        //    std::cout<<"Start "<<std::endl;

            double newError = maxError;
            if (maxError > error)
            {
                /*  If error not too large, try some reparameterization  */
                /*  and iteration */
                for (auto i = 0; i < maxIterations; i++)
                {
                    auto uPrime = BezierRep::Reparameterize(d, first, last, u, bezCurve);   //  Improved parameter values
                    GenerateBezier(d, first, last, uPrime, tHat1, tHat2, bezCurve);
        //            maxError = BezierRep::ComputeMaxError(d, first, last, bezCurve, uPrime, &splitPoint2D);
                    maxError = BezierRep::ComputeAvgError(d, first, last, bezCurve, uPrime);
                    if (maxError < newError)
                    {
                        newError = maxError;
                        result[result.size()-3] = bezCurve[1];
                        result[result.size()-2] = bezCurve[2];
                        result[result.size()-1] = bezCurve[3];
                    }
                    u = uPrime;
                }
            }
#if 1
            if (newError > error && splitPoint2D > first && splitPoint2D < last-1)
            {
                /* Fitting failed -- split at max error point and fit recursively */
                result.pop_back();
                result.pop_back();
                result.pop_back();
                
                auto tHatCenter = ComputeRightTangent(d, splitPoint2D);
                FitCubic(d, first, splitPoint2D, tHat1, tHatCenter, error, result);
                tHatCenter = ComputeLeftTangent(d, splitPoint2D);
                FitCubic(d, splitPoint2D, last, tHatCenter, tHat2, error, result);
            }
#endif
        }
        static std::vector<geom::Vector2f> FitCurve(const geom::Vector2f d[], size_t dSize, double error)
        {
            auto tHat1 = ComputeLeftTangent(d, 0); //  Unit tangent vectors at endpoints
            auto tHat2 = ComputeRightTangent(d, dSize-1);
            std::vector<geom::Vector2f> result;
            result.push_back(d[0]);
            FitCubic(d, 0, dSize-1, tHat1, tHat2, error, result);
            return result;
        }
        std::vector<geom::Vector2f> BezierRep::FitCurve(const std::vector<geom::Vector2f> &d, double error)
        {
            return extensions::puppet::FitCurve(d.data(), d.size(), error);
        }
        std::vector<double> BezierRep::Reparameterize( const geom::Vector2f d[], size_t first, size_t last,
                                                      const std::vector<double> &u, const geom::Vector2f bezCurve[4])
        {
            std::vector<double> uPrime(last - first + 1); //  New parameter values

            for (auto i = first; i <= last; i++)
            {
                uPrime[i - first] = NewtonRaphsonRootFind(bezCurve, d[i], u[i - first]);
            }
            return uPrime;
        }

        double BezierRep::ComputeAvgError(const geom::Vector2f d[], size_t first, size_t last, const geom::Vector2f bezCurve[4], const std::vector<double> &u)
        {
            double    avgDist      = 0.0;
            for (auto i = first + 1; i < last; i++)
            {
                double P[2];  //  point on curve
                EvalBezierFast(bezCurve, u[i-first], P);
                double dx = P[0] - d[i].x;//  offset from point to curve
                double dy = P[1] - d[i].y;
                auto dist = sqrt(dx*dx+dy*dy); //  Current error
                avgDist += dist;
            }
            return avgDist/(last-first+1);
        }
        double BezierRep::ComputeMaxError(const geom::Vector2f d[], size_t first, size_t last, const geom::Vector2f bezCurve[4],
                                          const std::vector<double> &u, size_t *splitPoint2D)
        {
            double    maxDist; //  Maximum error

            if (splitPoint2D)
                *splitPoint2D = (last - first + 1) / 2;
            maxDist      = 0.0;
            for (auto i = first + 1; i < last; i++)
            {
                double P[2];  //  point on curve
                EvalBezierFast(bezCurve, u[i-first], P);
                double dx = P[0] - d[i].x;//  offset from point to curve
                double dy = P[1] - d[i].y;
                auto dist = sqrt(dx*dx+dy*dy); //  Current error
                if (dist >= maxDist)
                {
                    maxDist = dist;
                    if (splitPoint2D)
                        *splitPoint2D = i;
                }
            }
            return maxDist;
        }
        std::vector<double> BezierRep::ChordLengthParameterize(const geom::Vector2f d[], size_t first, size_t last)
        {
            std::vector<double> u(last-first+1);//  Parameterization
            
            double prev = 0.0;
            u[0] = prev;
            for (auto i = first + 1; i <= last; i++)
            {
                auto & lastd = d[i-1];
                auto & curd  = d[i];
                auto dx = lastd.x - curd.x;
                auto dy = lastd.y - curd.y;
                prev = u[i - first] = prev + sqrt(dx*dx+dy*dy);
            }
                
            double ulastfirst = u[last-first];
            for (auto i = first + 1; i <= last; i++)
            {
                u[i - first] /= ulastfirst;
            }

            return u;
        }
    }
}
