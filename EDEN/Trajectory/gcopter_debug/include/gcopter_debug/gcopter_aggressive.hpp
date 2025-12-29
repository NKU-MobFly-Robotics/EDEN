/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef GCOPTER_AGGRESSIVE_HPP
#define GCOPTER_AGGRESSIVE_HPP

#include "gcopter_debug/minco.hpp"
#include "gcopter_debug/geo_utils.hpp"
#include "gcopter_debug/lbfgs.hpp"
#include "gcopter_debug/fastcheck_traj.hpp"

#include <Eigen/Eigen>

#include<fstream>
#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

using namespace std;
namespace gcopter_aggressive
{
    class GCOPTER_AGGRESSIVE
    {
    public:
        typedef Eigen::Matrix3Xd PolyhedronV;
        typedef Eigen::MatrixX4d PolyhedronH;
        typedef std::vector<PolyhedronV> PolyhedraV;
        typedef std::vector<PolyhedronH> PolyhedraH;

    private:
        // minco::MINCO_S3NU minco;
        minco::MINCO_S4NU_BR minco_br;
        // flatness::FlatnessMap flatmap;
        int c_;
        double rho;
        Eigen::Matrix3Xd vVel;
        Eigen::Matrix3Xd vAcc;
        Eigen::Matrix3Xd vPos;
        Eigen::Vector2d Dy;
        Eigen::Vector2d Ddy;


        Eigen::Matrix4Xd headPVA; // init
        Eigen::Matrix4Xd mainPVA; // main end
        Eigen::Matrix4Xd subPVA;  // sub end
        Eigen::Matrix4Xd stemPVA; // stem end

        // Eigen::Matrix3Xd vEndVel;
        PolyhedraV vPolytopesStem;
        PolyhedraH hPolytopesStem;
        PolyhedraV vPolytopesMain;
        PolyhedraH hPolytopesMain;
        PolyhedraV vPolytopesSub;
        PolyhedraH hPolytopesSub;
        PolyhedronH hPolytopesCam;
        // std::vector<Trajectory4<5>> *swarm_trajs_;
        SWARM_TRAJs *swarmTrajs_;
        
        Eigen::Matrix3Xd shortPathStem;
        Eigen::Matrix3Xd shortPathMain;
        Eigen::Matrix3Xd shortPathSub;
        Eigen::VectorXd yawPathStem;
        Eigen::VectorXd yawPathMain;
        Eigen::VectorXd yawPathSub;
        std::vector<std::pair<double, double>> yawRangeStem;
        std::vector<std::pair<double, double>> yawRangeMain;
        std::vector<std::pair<double, double>> yawRangeSub;

        Eigen::VectorXi pieceIdxSub;
        Eigen::VectorXi vPolyIdxSub;
        Eigen::VectorXi hPolyIdxSub;
        Eigen::VectorXi pieceIdxStem; 
        Eigen::VectorXi vPolyIdxStem;
        Eigen::VectorXi hPolyIdxStem;
        Eigen::VectorXi pieceIdxMain; 
        Eigen::VectorXi vPolyIdxMain;
        Eigen::VectorXi hPolyIdxMain;

        int pieceNSub;
        int pieceNStem;
        int pieceNMain;

        int spatialDimStem;
        int temporalDimStem;
        int yawDimStem;
        int spatialDimMain;
        int temporalDimMain;
        int yawDimMain;
        int spatialDimSub;
        int temporalDimSub;
        int yawDimSub;
        int VelDim;
        int AccDim;
        int PosDim;



        double smoothEps;
        int integralRes;
        Eigen::VectorXd magnitudeBd;
        Eigen::VectorXd penaltyWt;
        // Eigen::VectorXd physicalPm;
        double allocSpeed, allocYawSpeed;

        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix4Xd pointsStem; //stem interpoints
        Eigen::Matrix4Xd pointsSub; //sub interpoints
        Eigen::Matrix4Xd pointsMain; //main interpoints
        Eigen::VectorXd timesStem;      // stem times
        Eigen::VectorXd timesSub;      // sub times
        Eigen::VectorXd timesMain;      // main times
        Eigen::Matrix4Xd gradByPointsStem; 
        Eigen::Matrix4Xd gradByPointsSub; 
        Eigen::Matrix4Xd gradByPointsMain; 
        Eigen::VectorXd gradByTimesStem;
        Eigen::VectorXd gradByTimesSub;
        Eigen::VectorXd gradByTimesMain;
        Eigen::Vector4d gradByStemVel;
        Eigen::Vector4d gradByStemAcc;
        Eigen::Vector4d gradByMainVel;
        Eigen::Vector4d gradByMainAcc;

        Eigen::MatrixX4d partialGradByCoeffsSub; 
        Eigen::MatrixX4d partialGradByCoeffsStem; 
        Eigen::MatrixX4d partialGradByCoeffsMain; 
        Eigen::VectorXd partialGradByTimesSub; 
        Eigen::VectorXd partialGradByTimesStem; 
        Eigen::VectorXd partialGradByTimesMain; 

    private:
        /**
         * @brief get T from tau
         * 
         * @param tau \in R^n
         * @param T   the interval of each segment
         */
        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &T)
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        /**
         * @brief  map T to tau
         *      since optimizer wishes to optimize tau without constraints, while T >= 0.
         *      tau > 0: T = 1/2 tau^2 + tau + 1; tau <= 0: T = 2/(tau^2 - tau + 1); R ---> R+
         * @tparam EIGENVEC 
         * @param T   the interval of each segment
         * @param tau \in R^n
         */
        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                     EIGENVEC &tau)
        {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0
                             ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                             : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        /**
         * @brief get the grad of tau using grad of T
         * 
         * @tparam EIGENVEC 
         * @param tau 
         * @param gradT 
         * @param gradTau 
         */
        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau)
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }
            return;
        }

        /**
         * @brief get mid points(mid points are neccessary for formulating MINCO)
         * 
         * @param xi        the weight of each vector 
         * @param vIdx      the index of corresponding vPolys(path is shortten, some vPolys are abandoned)
         * @param vPolys    the vector expression of polytopes([start vetex, vector_1, ..., vectorn_n)
         * @param yRange    the yaw corridor [y0-1.5, y0+1.5]
         * @param yi        the weight of each yaw vector 
         * 
         * @param P         the out put mid points
         */
        static inline void forwardP(const Eigen::VectorXd &xi,
                                    const Eigen::VectorXi &vIdx,
                                    const PolyhedraV &vPolys,
                                    const std::vector<std::pair<double, double>> &yRange,
                                    const Eigen::VectorXd &yi,
                                    Eigen::Matrix4Xd &P)
        {
            const int sizeP = vIdx.size();
            // std::cout<<"forwardP0"<<std::endl;
            P.resize(4, sizeP);
            // std::cout<<"sizeP:"<<sizeP<<std::endl;

            Eigen::VectorXd q;
            double qy;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                // std::cout<<"l:"<<l<<std::endl;
                // std::cout<<"vPolys:"<<vPolys.size()<<std::endl;
                k = vPolys[l].cols();
                // std::cout<<"xi:"<<xi.size()<<std::endl;
                // std::cout<<"j:"<<j<<" k:"<<k<<std::endl;
                q = xi.segment(j, k).normalized().head(k - 1);
                // std::cout<<"P:"<<P.cols()<<std::endl;
                P.col(i).head(3) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                           vPolys[l].col(0);
                // std::cout<<"yi:"<<yi.size()<<std::endl;
                qy = yi.segment(i * 2, 2).normalized()(0);
                // P.col(i)(3) = yi(i);
                // if(i >= yRange.size()) std::cout<<"???"<<std::endl;
                P.col(i)(3) = yRange[i].second * qy * qy + yRange[i].first;
            }
            return;
        }

        static inline void forwardVA(const Eigen::Matrix3Xd &vVelocity,
            const Eigen::VectorXd &evi,
            const Eigen::Vector2d &vyaw,
            const Eigen::VectorXd &eyawi,
            Eigen::Vector3d &endv,
            double &endDy){
            const int k = vVelocity.cols();
            Eigen::VectorXd q = evi.normalized().head(k-1);
            endv = vVelocity.rightCols(k-1) * q.cwiseProduct(q) + vVelocity.col(0);
            double qy = eyawi.normalized()(0);
            endDy = vyaw(0) + vyaw(1) * qy * qy;
        }

        /**
         * @brief the interface of BFGS
         * 
         * @param ptr 
         * @param xi     optimization variable
         * @param gradXi 
         * @return objective cost 
         */
        static inline double costTinyNLS(void *ptr,
                                         const Eigen::VectorXd &xi,
                                         Eigen::VectorXd &gradXi)
        {
            const int n = xi.size();
            const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

            const double sqrNormXi = xi.squaredNorm();
            const double invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi = xi * invNormXi;
            const Eigen::VectorXd r = unitXi.head(n - 1);
            const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                          ovPoly.col(1) - ovPoly.col(0);

            double cost = delta.squaredNorm();
            gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                                 r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double c = sqrNormViolation * sqrNormViolation;
                const double dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi += dc * 2.0 * xi; 
            }

            return cost; 
        }

        /**
         * @brief map mid points to weights of polytope vectors
         * since mid points are constrained in convex hulls, while optimizer wish free optimization, square weights are used
         * besides, polytope may not be a simplex，optimization is used in this function
         * 
         * @tparam EIGENVEC 
         * @param P         mid points
         * @param vIdx      index of corresponding polytope
         * @param vPolys    all the polytopes
         * @param xi        weights
         */
        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix4Xd &P,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double minSqrD;
            lbfgs::lbfgs_parameter_t tiny_nls_params;
            tiny_nls_params.past = 0;
            tiny_nls_params.delta = 1.0e-5;
            tiny_nls_params.g_epsilon = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix3Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols(); 
                //ovPoly: [pathpoint vertex1 ... vertexk]
                ovPoly.resize(3, k + 1);
                ovPoly.col(0) = P.col(i).head(3);
                ovPoly.rightCols(k) = vPolys[l];
                Eigen::VectorXd x(k);
                x.setConstant(sqrt(1.0 / k));
                lbfgs::lbfgs_optimize(x,
                                      minSqrD,
                                      &GCOPTER_AGGRESSIVE::costTinyNLS,
                                      nullptr,
                                      nullptr,
                                      &ovPoly,
                                      tiny_nls_params);

                xi.segment(j, k) = x;
            }

            return;
        }

        /**
         * @brief get the grad of weights
         * 
         * @tparam EIGENVEC 
         * @param xi        weights 
         * @param vIdx      index of corresponding polytope
         * @param vPolys    all the polytopes
         * @param yi        yaw weights 
         * @param yRange    the yaw corridor [y0-1.5, y0+1.5]
         * @param gradP     grad of mid points
         * @param gradXi    grad of weights
         * @param gradYi    grad of yaw weights
         */
        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::VectorXd &yi,
                                         const std::vector<std::pair<double, double>> &yRange,
                                         const Eigen::Matrix4Xd &gradP,
                                         EIGENVEC &gradXi,
                                         EIGENVEC &gradYi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double normInv;
            Eigen::VectorXd q, gradQ, unitQ;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k);
                normInv = 1.0 / q.norm();
                unitQ = q * normInv;
                gradQ.resize(k);
                gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i).head(3)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;

                q = yi.segment(i*2, 2);
                normInv = 1.0 / q.norm();
                unitQ = q * normInv;
                gradQ.resize(2);
                gradQ(0) = (yRange[i].second * gradP.col(i)(3)) *
                                    unitQ(0) * 2.0;
                gradQ(1) = 0.0;
                gradYi.segment(i * 2, 2) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
                // gradYi(i) = gradP.col(i)(3);
                // if(i >= gradYi.size()) std::cout<<"gradYi out!!!!!!!!"<<std::endl;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradVA(const Eigen::VectorXd &gradv,
                                         const Eigen::VectorXd &vi,
                                         const Eigen::Matrix3Xd &vVelocity,
                                         const Eigen::VectorXd &dyi,
                                         const Eigen::Vector2d &vDy,
                                         EIGENVEC &gradVi,
                                         EIGENVEC &gradDy){
            gradVi.resize(vi.size());
            Eigen::VectorXd q, gradQ, unitQ;
            double normInv;
            int k = vVelocity.cols();
            q = vi;
            normInv = 1.0 / q.norm();
            unitQ = q * normInv;
            gradQ.resize(k);
            gradQ.head(k - 1) = (vVelocity.rightCols(k - 1).transpose() * gradv.head(3)).array() *
                                unitQ.head(k - 1).array() * 2.0;
            gradQ(k - 1) = 0.0;
            gradVi = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;

            gradDy.resize(dyi.size());
            k = 2;
            q = dyi;
            normInv = 1.0 / q.norm();
            unitQ = q * normInv;
            gradQ.resize(k);
            gradQ(0) = (vDy(1) * gradv(3)) *
                                unitQ(0) * 2.0;
            gradQ(1) = 0.0;
            gradDy = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;

        }

        /**
         * @brief a special cost function,  stabilize the norm of xi at 1 
         * 
         * @tparam EIGENVEC 
         * @param xi 
         * @param vIdx 
         * @param vPolys 
         * @param cost 
         * @param gradXi 
         */
        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                               const Eigen::VectorXi &vIdx,
                                               const PolyhedraV &vPolys,
                                               const Eigen::VectorXd &yi,
                                               double &cost,
                                               EIGENVEC &gradXi, 
                                               EIGENVEC &gradYi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            Eigen::VectorXd qy;
            for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
            {
                k = vPolys[vIdx(i)].cols();

                q = xi.segment(j, k);
                sqrNormQ = q.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }

                qy = yi.segment(i*2, 2);
                sqrNormQ = qy.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradYi.segment(i*2, 2) += dc * 2.0 * qy;
                }
            }

            return;
        }


        template <typename EIGENVEC>
        static inline void normRetrictionLayerVA(const Eigen::VectorXd &vi,
                                                const Eigen::VectorXd &dyi,
                                               double &cost,
                                               EIGENVEC &gradVi,
                                               EIGENVEC &gradDyi)
        {
            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            q = vi;
            sqrNormQ = q.squaredNorm();
            sqrNormViolation = sqrNormQ - 1.0;
            if (sqrNormViolation > 0.0)
            {
                c = sqrNormViolation * sqrNormViolation;
                dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradVi += dc * 2.0 * q;
            }

            q = dyi;
            sqrNormQ = q.squaredNorm();
            sqrNormViolation = sqrNormQ - 1.0;
            if (sqrNormViolation > 0.0)
            {
                c = sqrNormViolation * sqrNormViolation;
                dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradDyi += dc * 2.0 * q;
            }
            return;
        }

        /**
         * @brief a smooth punish function, punish if x > mu
         * 
         * @param x 
         * @param mu    the punish threshold
         * @param f     punish cost
         * @param df    grad of x
         * @return true 
         * @return false 
         */
        static inline bool smoothedL1(const double &x,
                                      const double &mu,
                                      double &f,
                                      double &df)
        {
            if (x < 0.0)
            {
                return false;
            }
            else if (x > mu)
            {
                f = x - 0.5 * mu;
                df = 1.0;
                return true;
            }
            else
            {
                const double xdmu = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2 = mu - 0.5 * x;
                f = mumxd2 * sqrxdmu * xdmu;
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
                return true;
            }
        }

        /**
         * @brief user define discrete cost. for inequality constraint 
         * and integral functions that are difficult to be expressed analytically 
         * 
         * @param T                     intervals
         * @param coeffs                polynomial params
         * @param hIdx                  index of corresponding polytopes
         * @param hPolys                all the polytopes
         * @param smoothFactor          the mu in smoothedL1
         * @param integralResolution    the number of discrete pieces
         * @param magnitudeBounds       user define, here [v_max, a_max,]^T 
         * @param penaltyWeights        [pos_weight, vel_weight, acc_weight]^T
         * @param cost                  objective cost
         * @param gradT                 grad of intervals
         * @param gradC                 grad of coeffs
         */
        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                const Eigen::MatrixX4d &coeffs,
                                                const Eigen::VectorXi &hIdx,
                                                const PolyhedraH &hPolys,
                                                const PolyhedronH &CamhPolys,
                                                const double &smoothFactor,
                                                const int &integralResolution,
                                                const Eigen::VectorXd &magnitudeBounds,
                                                const Eigen::VectorXd &penaltyWeights,
                                                const std::vector<std::vector<int>> &seg_intersec_ids,
                                                SWARM_TRAJs *swarmTrajs,
                                                // const vector<Trajectory4<5>> *&swarmTrajs,
                                                // const vector<pair<double, double>> &swarmTrajsStartEndTimes,
                                                // const vector<Eigen::Vector3d> &swarmPos,
                                                const double &t0,
                                                double &cost,
                                                Eigen::VectorXd &gradT,
                                                Eigen::MatrixX4d &gradC,
                                                bool mainbranch){

            const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
            const double accSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
            const double jerSqrMax = magnitudeBounds(2) * magnitudeBounds(2);
            const double dyawMax = magnitudeBounds(3) * magnitudeBounds(3);
            const double ddyawMax = magnitudeBounds(4) * magnitudeBounds(4);
            const double swarmSqrMax = magnitudeBounds(5) * magnitudeBounds(5);

            const double weightPos = penaltyWeights(0);
            const double weightVel = penaltyWeights(1);
            const double weightAcc = penaltyWeights(2);
            const double weightJer = penaltyWeights(3);
            const double weightDyaw = penaltyWeights(4);
            const double weightDdyaw = penaltyWeights(5);
            const double weightSwarm = penaltyWeights(6);
            // const double weightCoverVel = penaltyWeights(7);

            // const double weightJer = penaltyWeights(6);

            Eigen::Vector4d pos, vel, acc, jer, sna;
            Eigen::Vector4d gradPos, gradVel, gradAcc, gradJer;
            Eigen::Vector3d pSwarm, vSwarm, dps;


            double step, alpha, tse, tc, tsi;
            double s1, s2, s3, s4, s5;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d outerNormal;
            int K, L, /*D,*/ S;
            double node, pena;
            double violaPos, violaVel, violaAcc, violaJer, violaDyaw, violaDdyaw, violaSwarm;//, violaVc;
            double violaPosPena, violaVelPena, violaAccPena, violaJerPena, violaDyawPena, violaDdyawPena, violaSwarmPena;//, violaVcPena;
            double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaJerPenaD, violaDyawPenaD, violaDdyawPenaD, violaSwarmPenaD;//, violaVcPenaD;

            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;
            
            // D = swarmTrajs->trajs_.size();
            S = swarmTrajs->SwarmPos_.size();
            tse = 0.0;



            Eigen::Vector4d q, y_d_q, gradQ;
            Eigen::Vector3d az, z, vc, gradVc, gradZ, gradAccCov;
            Eigen::Matrix3d W2C; // world to camera
            Eigen::Matrix3d z_d_a;
            Eigen::Matrix<double, 4, 3> Vc_d_q;
            Eigen::Matrix<double, 3, 4> q_d_z;
            Eigen::Vector3d v;
            // double gradYaw;
            
            // int M = CamhPolys.rows();
            // double sqrt_2_1pz2, sqrt_2_1pz2_inv, yaw2, siny2, cosy2, cosy2_d_y, siny2_d_y;
            // double anorm, anorm3, anorm3_inv, anorm_inv, m11, m12, m13, m22, m23, m33;
            // // double pen; // len, len_inv;
            // double qxy, qxz, qxw, qyz, qyw, qzw, qxx, qyy, qzz;// qww; 
            // bool fail_cover;

            for (int i = 0; i < pieceNum; i++)
            {
                const Eigen::Matrix<double, 6, 4> &c = coeffs.block<6, 4>(i * 6, 0);
                step = T(i) * integralFrac;
                for (int j = 0; j <= integralResolution; j++)
                {
                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                    alpha = j * integralFrac;

                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;

                    // az = acc.head(3);
                    // az(2) += 9.8;
                    // z = az.normalized();

                    violaVel = vel.head(3).squaredNorm() - velSqrMax;
                    violaAcc = acc.head(3).squaredNorm() - accSqrMax;
                    violaJer = jer.head(3).squaredNorm() - jerSqrMax;
                    violaDyaw = vel(3) * vel(3) - dyawMax;
                    violaDdyaw = acc(3) * acc(3) - ddyawMax;
                    L = hIdx(i);
                    K = hPolys[L].rows();

                    gradPos.setZero(), gradVel.setZero(), gradAcc.setZero(), gradJer.setZero(), gradVc.setZero();
                    pena = 0.0;

                    tc = tse + j * step + t0;

                    // main branch cost
                    // if(mainbranch){
                    //     az = acc.head(3);
                    //     az(2) += 9.8;
                    //     z = az.normalized();
                    //     sqrt_2_1pz2 = sqrt(2*(1 + z(2)));
                    //     sqrt_2_1pz2_inv = 1.0 / sqrt_2_1pz2;
                    //     yaw2 = pos(3) * 0.5;
                    //     siny2 = sin(yaw2);
                    //     cosy2 = cos(yaw2);
                    //     q(0) = (1 + z(2))*cosy2;
                    //     q(1) = -z(1)*cosy2 + z(0)*siny2;
                    //     q(2) = z(0)*cosy2 + z(1)*siny2;
                    //     q(3) = (1 + z(2))*siny2;
                    //     q *= sqrt_2_1pz2_inv;

                    //     q_d_z.setZero();
                    //     q_d_z(0, 1) = siny2 * sqrt_2_1pz2_inv;
                    //     q_d_z(0, 2) = cosy2 * sqrt_2_1pz2_inv;
                    //     q_d_z(1, 1) = -cosy2 * sqrt_2_1pz2_inv;
                    //     q_d_z(1, 2) = siny2 * sqrt_2_1pz2_inv;
                    //     q_d_z(2, 1) = -q(1) * sqrt_2_1pz2_inv * sqrt_2_1pz2_inv;
                    //     q_d_z(2, 2) = -q(2) * sqrt_2_1pz2_inv * sqrt_2_1pz2_inv;
                    //     q_d_z(2, 0) = cosy2 * sqrt_2_1pz2_inv * 0.5;
                    //     q_d_z(2, 3) = siny2 * sqrt_2_1pz2_inv * 0.5;

                    //     qxy = q(1) * q(2);
                    //     qxz = q(1) * q(3);
                    //     qxw = q(1) * q(0);
                    //     qyz = q(2) * q(3);
                    //     qyw = q(2) * q(0);
                    //     qzw = q(3) * q(0);
                    //     qxx = q(1) * q(1);
                    //     qyy = q(2) * q(2);
                    //     qzz = q(3) * q(3);

                    //     W2C <<  1-2*(qyy+qzz), 2*(qxy + qzw), 2*(qxz - qyw),
                    //             2*(qxy - qzw), 1-2*(qxx+qzz), 2*(qyz + qxw),
                    //             2*(qxz + qyw), 2*(qyz - qxw), 1-2*(qxx+qyy);

                    //     cosy2_d_y = -siny2*0.5;
                    //     siny2_d_y = cosy2*0.5;
                    //     y_d_q(0) = (1 + z(2)) * cosy2_d_y;
                    //     y_d_q(1) = -z(1)*cosy2_d_y + z(0)*siny2_d_y;
                    //     y_d_q(2) = z(0)*cosy2_d_y + z(1)*siny2_d_y;
                    //     y_d_q(3) = (1 + z(2))*siny2_d_y;
                    //     y_d_q *= sqrt_2_1pz2_inv;
                        
                    //     anorm = az.norm();
                    //     anorm3 = anorm * anorm * anorm;
                    //     anorm_inv = 1.0 / anorm;
                    //     anorm3_inv = 1.0 / anorm3;
                    //     m11 =anorm_inv - az(0)*az(0)*anorm3_inv, m22 = anorm_inv - az(1)*az(1)*anorm3_inv, m33 = anorm_inv - az(2)*az(2)*anorm3_inv,
                    //         m12 = -az(0)*az(1)*anorm3_inv, m13 = -az(0)*az(2)*anorm3_inv, m23 = -az(1)*az(2)*anorm3_inv;
                    //     z_d_a << m11, m12, m13,
                    //             m12, m22, m23, 
                    //             m13, m23, m33;   
                    //     v = vel.head(3);
                    //     vc = W2C * v;
                    //     fail_cover = false;
                    //     for(int n = 0; n < M; n++){
                    //         outerNormal = CamhPolys.block<1, 3>(n, 0);
                    //         violaVc = outerNormal.dot(vc.head(3)) + CamhPolys(n, 3);
                    //         if (smoothedL1(violaVc, smoothFactor, violaVcPena, violaVcPenaD)){

                    //             fail_cover = true;
                    //             gradVc += weightCoverVel * violaVcPenaD * outerNormal;
                    //             gradVel.head(3) += W2C * weightCoverVel * violaVcPenaD * outerNormal;
                    //             pena += weightCoverVel * violaVcPena;
                    //         }
                    //     }
                    //     if(fail_cover){
                    //         // if(!fail_cover) continue;

                    //         Vc_d_q <<   2*(q(3)*v(1) - q(2)*v(2))             , 2*(-q(3)*v(0) + q(1)*v(2))               , 2*(q(2)*v(0)-q(1)*v(1)),
                    //                     2*(q(2)*v(1) + q(3)*v(2))             , 2*(q(2)*v(0) - 2*q(1)*v(1) + q(0)*v(2)), 2*(q(3)*v(0)-q(0)*v(1)-2*q(1)*v(2)),
                    //                     2*(-2*q(2)*v(0)+q(1)*v(1)-q(0)*v(2)), 2*(q(1)*v(0) + q(3)*v(2))                , 2*(q(0)*v(0)+q(3)*v(1)-2*q(2)*v(2)),
                    //                     2*(-2*q(3)*v(0)+q(0)*v(1)+q(1)*v(2)), 2*(-q(0)*v(0) -2*q(3)*v(1) + q(2)*v(2)), 2*(q(1)*v(0)+q(2)*v(1));
                    //         gradQ = Vc_d_q * gradVc;
                    //         gradZ = q_d_z * gradQ;
    
                    //         gradYaw = gradQ.transpose() * y_d_q;
    

                    //         gradPos(3) += gradYaw;
    
                    //         gradAcc.head(3) += z_d_a * gradZ;
                    //         std::cout<<"v:"<<v.transpose()<<endl;
                    //         std::cout<<"vc:"<<vc.transpose()<<endl;
                    //         std::cout<<"C2W:\n"<<W2C.transpose()<<endl;
                    //         std::cout<<"a:"<<acc.transpose()<<endl;
                    //         std::cout<<"y:"<<pos(3)<<endl;
                    //     }
                    // }



                    // swarm traj cost
                    // if(i >= seg_intersec_ids.size()){
                    //     std::cout<<"out i"<<std::endl;
                    //     getchar();
                    // }
                    for(auto &id : seg_intersec_ids[i]){
                        // std::cout<<"id:"<<id<<std::endl;
                        // std::cout<<"swarmTrajs->Ts_:"<<swarmTrajs->Ts_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->trajs_:"<<swarmTrajs->trajs_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->nVs_:"<<swarmTrajs->nVs_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->durations_:"<<swarmTrajs->durations_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->Ve_:"<<swarmTrajs->Ve_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->Pe_:"<<swarmTrajs->Pe_.size()<<std::endl;
                        tsi = tc - swarmTrajs->Ts_.at(id);
                        if(tsi < 0){
                            pSwarm = swarmTrajs->Ps_.at(id) + tsi * swarmTrajs->nVs_.at(id);
                            vSwarm = -swarmTrajs->nVs_.at(id);
                        }
                        else if(tsi < swarmTrajs->durations_.at(id)){
                            pSwarm = swarmTrajs->trajs_.at(id).getPos(tsi).head(3);
                            vSwarm = swarmTrajs->trajs_.at(id).getVel(tsi).head(3);
                        }
                        else{
                            pSwarm = swarmTrajs->Pe_.at(id) + (tsi - swarmTrajs->durations_.at(id)) * swarmTrajs->Ve_.at(id);
                            vSwarm = swarmTrajs->Ve_.at(id);
                        }
                        dps = pSwarm - pos.head(3);
                        violaSwarm = swarmSqrMax - (dps).squaredNorm();
                        if (smoothedL1(violaSwarm, smoothFactor, violaSwarmPena, violaSwarmPenaD))
                        {
                            gradPos.head(3) += 2.0 * weightSwarm * violaSwarmPenaD * dps;
                            // std::cout<<"swarm collide pos:"<<pos.transpose()<<std::endl;
                            // std::cout<<"swarm collide pSwarm:"<<pSwarm.transpose()<<std::endl;

                            pena += weightSwarm * violaSwarmPena;
                            gradT(i) -= ((2.0 * weightSwarm * violaSwarmPenaD) * dps.dot(vSwarm)) *
                               alpha * node * step;
                        }
                    }

                    // swarm pos cost
                    for(int id = 0; id < S; id++){
                        dps = swarmTrajs->SwarmPos_.at(id) - pos.head(3);
                        violaSwarm = swarmSqrMax - (dps).squaredNorm();
                        if (smoothedL1(violaSwarm, smoothFactor, violaSwarmPena, violaSwarmPenaD))
                        {
                            gradPos.head(3) += 2.0 * weightSwarm * violaSwarmPenaD * dps;
                            pena += weightSwarm * violaSwarmPena;
                        }
                    }

                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = hPolys[L].block<1, 3>(k, 0);
                        violaPos = outerNormal.dot(pos.head(3)) + hPolys[L](k, 3);
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            gradPos.head(3) += weightPos * violaPosPenaD * outerNormal;
                            pena += weightPos * violaPosPena;
                        }
                    }
                    if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                    {
                        gradVel.head(3) += weightVel * violaVelPenaD * 2.0 * vel.head(3);
                        pena += weightVel * violaVelPena;
                    }
                    if (smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD))
                    {
                        gradAcc.head(3) += weightAcc * violaAccPenaD * 2.0 * acc.head(3);
                        pena += weightAcc * violaAccPena;
                    }
                    if (smoothedL1(violaJer, smoothFactor, violaJerPena, violaJerPenaD))
                    {
                        gradJer.head(3) += weightJer * violaJerPenaD * 2.0 * jer.head(3);
                        pena += weightJer * violaJerPena;
                    }

                    if (smoothedL1(violaDyaw, smoothFactor, violaDyawPena, violaDyawPenaD))
                    {
                        gradVel(3) += weightDyaw * violaDyawPenaD * 2.0 * vel(3);
                        pena += weightDyaw * violaDyawPena;
                    }
                    if (smoothedL1(violaDdyaw, smoothFactor, violaDdyawPena, violaDdyawPenaD))
                    {
                        gradAcc(3) += weightDdyaw * violaDdyawPenaD * 2.0 * acc(3);
                        pena += weightDdyaw * violaDdyawPena;
                    }


                    gradC.block<6, 4>(i * 6, 0) += (beta0 * gradPos.transpose() +
                                                    beta1 * gradVel.transpose() +
                                                    beta2 * gradAcc.transpose() +
                                                    beta3 * gradJer.transpose()) *
                                                   node * step;
                    gradT(i) += (gradPos.dot(vel) +
                                 gradVel.dot(acc) +
                                 gradAcc.dot(jer) +
                                 gradJer.dot(sna)) *
                                    alpha * node * step +
                                node * integralFrac * pena;
                    cost += node * step * pena;
                }
                tse += T(i);
            }
            return;
        }

        /**
         * @brief interface of BFGS, optimize MINCO
         * 
         * @param ptr 
         * @param x 
         * @param g 
         * @return objective cost
         */
        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {
            GCOPTER_AGGRESSIVE &obj = *(GCOPTER_AGGRESSIVE *)ptr;
            // const int dimTau = obj.temporalDim;
            // const int dimXi = obj.spatialDim;
            // const int dimEvi = obj.endVelDim;

            const int dTauSu = obj.temporalDimSub;
            const int dTauSt = obj.temporalDimStem;
            const int dTauMa = obj.temporalDimMain;
            const int dXSu = obj.spatialDimSub;
            const int dXSt = obj.spatialDimStem;
            const int dXMa = obj.spatialDimMain;
            const int dYSu = obj.yawDimSub;
            const int dYSt = obj.yawDimStem;
            const int dYMa = obj.yawDimMain;
            // const int dVel = obj.VelDim;
            // const int dAcc = obj.AccDim;


            const double weightT = obj.rho;
            // double fminT, dfminT;

            int s = 0;
            Eigen::Map<const Eigen::VectorXd> tauSub(x.data(), dTauSu);
            s += dTauSu;
            Eigen::Map<const Eigen::VectorXd> tauStem(x.data() + s, dTauSt);
            s += dTauSt;
            Eigen::Map<const Eigen::VectorXd> tauMain(x.data() + s, dTauMa);
            s += dTauMa;
            
            Eigen::Map<const Eigen::VectorXd> xiSub(x.data() + s, dXSu);
            s += dXSu;
            Eigen::Map<const Eigen::VectorXd> xiStem(x.data() + s, dXSt);
            s += dXSt;
            Eigen::Map<const Eigen::VectorXd> xiMain(x.data() + s, dXMa);
            s += dXMa;

            Eigen::Map<const Eigen::VectorXd> yiSub(x.data() + s, dYSu);
            s += dYSu;
            Eigen::Map<const Eigen::VectorXd> yiStem(x.data() + s, dYSt);
            s += dYSt;
            Eigen::Map<const Eigen::VectorXd> yiMain(x.data() + s, dYMa);
            s += dYMa;
            Eigen::Map<const Eigen::VectorXd> vsi(x.data() + s, 4);
            s += 4;
            Eigen::Map<const Eigen::VectorXd> asi(x.data() + s, 4);
            s += 4;
            Eigen::Map<const Eigen::VectorXd> vmi(x.data() + s, 4);
            s += 4;
            Eigen::Map<const Eigen::VectorXd> ami(x.data() + s, 4);



            // Eigen::Map<const Eigen::VectorXd> vi(x.data() + s, dVel);
            // s += dVel;
            // Eigen::Map<const Eigen::VectorXd> dyi(x.data() + s, 2);
            // s += 2;
            // Eigen::Map<const Eigen::VectorXd> ai(x.data() + s, dAcc);
            // s += dAcc;
            // Eigen::Map<const Eigen::VectorXd> ddyi(x.data() + s, 2);

            // Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            // Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            // Eigen::Map<const Eigen::VectorXd> evi(x.data() + dimTau + dimXi, dimEvi);

            // Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
            // Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);
            // Eigen::Map<Eigen::VectorXd> gradEvi(g.data() + dimTau + dimXi, dimEvi);
            s = 0;
            Eigen::Map<Eigen::VectorXd> gradTauSub(g.data(), dTauSu);
            s += dTauSu;
            Eigen::Map<Eigen::VectorXd> gradTauStem(g.data() + s, dTauSt);
            s += dTauSt;
            Eigen::Map<Eigen::VectorXd> gradTauMain(g.data() + s, dTauMa);
            s += dTauMa;

            Eigen::Map<Eigen::VectorXd> gradXiSub(g.data() + s, dXSu);
            s += dXSu;
            Eigen::Map<Eigen::VectorXd> gradXiStem(g.data() + s, dXSt);
            s += dXSt;
            Eigen::Map<Eigen::VectorXd> gradXiMain(g.data() + s, dXMa);
            s += dXMa;

            Eigen::Map<Eigen::VectorXd> gradYiSub(g.data() + s, dYSu);
            s += dYSu;
            Eigen::Map<Eigen::VectorXd> gradYiStem(g.data() + s, dYSt);
            s += dYSt;
            Eigen::Map<Eigen::VectorXd> gradYiMain(g.data() + s, dYMa);
            s += dYMa;
            Eigen::Map<Eigen::VectorXd> gradVsi(g.data() + s, 4);
            s += 4;
            Eigen::Map<Eigen::VectorXd> gradAsi(g.data() + s, 4);
            s += 4;
            Eigen::Map<Eigen::VectorXd> gradVmi(g.data() + s, 4);
            s += 4;
            Eigen::Map<Eigen::VectorXd> gradAmi(g.data() + s, 4);


            
            // Eigen::Map<Eigen::VectorXd> gradVi(g.data() + s, dVel);
            // s += dVel;
            // Eigen::Map<Eigen::VectorXd> gradDyi(g.data() + s, 2);
            // s += 2;
            // Eigen::Map<Eigen::VectorXd> gradAi(g.data() + s, dAcc);
            // s += dAcc;
            // Eigen::Map<Eigen::VectorXd> gradDdyi(g.data() + s, 2);
            // Eigen::Vector3d endv;
            // forwardVA(obj.vEndVel, evi, endv);
            // forwardT(tau, obj.times);
            // forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

            forwardT(tauSub, obj.timesSub);
            forwardT(tauStem, obj.timesStem);
            forwardT(tauMain, obj.timesMain);
            forwardP(xiSub, obj.vPolyIdxSub, obj.vPolytopesSub, obj.yawRangeSub, yiSub, obj.pointsSub);
            forwardP(xiStem, obj.vPolyIdxStem, obj.vPolytopesStem, obj.yawRangeStem, yiStem, obj.pointsStem);
            forwardP(xiMain, obj.vPolyIdxMain, obj.vPolytopesMain, obj.yawRangeMain, yiMain, obj.pointsMain);
            // Eigen::Vector3d v;
            // double dy;
            // Eigen::Vector3d a;
            // double ddy;
            // forwardVA(obj.vVel, vi, obj.Dy, dyi, v, dy);
            // forwardVA(obj.vAcc, ai, obj.Ddy, ddyi, a, ddy);


            // forwardP(xiSub, obj.vPolyIdxSub, obj.vPolytopesSub, obj.pointsSub);
            // forwardP(xiStem, obj.vPolyIdxStem, obj.vPolytopesStem, obj.pointsStem);
            // forwardP(xiMain, obj.vPolyIdxMain, obj.vPolytopesMain, obj.pointsMain);
            // Eigen::VectorXd tMs(obj.pieceNStem + obj.pieceNMain);
            // tMs.head(obj.pieceNStem) = obj.timesStem;
            // tMs.head(obj.pieceNMain) = obj.timesMain;
            // Eigen::MatrixXd ptsMs(4, obj.pieceNMain + obj.pieceNStem - 1);
            // ptsMs.leftCols(obj.pieceNStem - 1) = obj.pointsStem;
            // ptsMs.col(obj.pieceNStem - 1) = obj.stemPVA.col(0);
            // ptsMs.rightCols(obj.pieceNMain - 1) = obj.pointsMain;

            // double cost;
            // obj.minco.setEndVel(endv);
            // obj.minco.setParameters(obj.points, obj.times);
            // obj.minco.getEnergy(cost);
            // obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            // obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
            // cost *= 0.1;
            // obj.partialGradByCoeffs *= 0.1;
            // obj.partialGradByTimes *= 0.1;
            obj.stemPVA.col(1) = vsi;
            obj.stemPVA.col(2) = asi;
            obj.mainPVA.col(1) = vmi;
            obj.mainPVA.col(2) = ami;
            // obj.stemPVA.col(1).head(3) = v;
            // obj.stemPVA(3, 1) = dy;
            // obj.stemPVA.col(2).head(3) = a;
            // obj.stemPVA(3, 2) = ddy;
            double cost;
            double cep = 0, ceym = 0, ceyst = 0, ceysu = 0;
            Eigen::MatrixX3d gpStem, gpMain, gpSub;
            Eigen::VectorXd gyStem, gyMain, gySub, gtStem, gtMain, gtSub;
            // minco_br.setConditions(headPVA, mainPVA, subPVA);
            obj.minco_br.setParameters(obj.stemPVA, obj.mainPVA, obj.pointsStem, obj.timesStem, obj.pointsMain, obj.timesMain, obj.pointsSub, obj.timesSub);
            // obj.minco_br.setParameters(obj.stemPVA, obj.pointsStem, obj.timesStem, obj.pointsMain, obj.timesMain, obj.pointsSub, obj.timesSub);
            obj.minco_br.getEnergyPos(cep);
            obj.minco_br.Main.getEnergyYaw(ceym);
            obj.minco_br.Stem.getEnergyYaw(ceyst);
            obj.minco_br.Sub.getEnergyYaw(ceysu);
            cost = cep + ceym * 0.25 + ceyst * 0.25 + ceysu * 0.0;
            obj.minco_br.getEnergyPartialGradByCoeffsPos(gpStem, gpMain, gpSub);
            obj.partialGradByCoeffsStem.leftCols(3) = gpStem;
            obj.partialGradByCoeffsMain.leftCols(3) = gpMain;
            obj.partialGradByCoeffsSub.leftCols(3) = gpSub*0.0;
            obj.minco_br.getEnergyPartialGradByCoeffsYaw(gyStem, gyMain, gySub);
            obj.partialGradByCoeffsStem.col(3) = gyStem * 0.25;
            obj.partialGradByCoeffsMain.col(3) = gyMain * 0.25;
            obj.partialGradByCoeffsSub.col(3) = gySub * 0.0;
            obj.minco_br.getEnergyPosPartialGradByTimes(gtStem, gtMain, gtSub);
            obj.partialGradByTimesStem = gtStem;
            obj.partialGradByTimesMain = gtMain;
            obj.partialGradByTimesSub = gtSub*0.0;
            obj.minco_br.getEnergyYawPartialGradByTimes(gtStem, gtMain, gtSub);
            obj.partialGradByTimesStem += gtStem * 0.25;
            obj.partialGradByTimesMain += gtMain * 0.25;
            obj.partialGradByTimesSub += gtSub * 0.0;
            // obj.minco_br.getEnergyPartialGradByTimes(cost);
            // obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            // obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);


            // if(obj.c_ == 0){
            //     std::cout<<"obj.pointsSub:\n"<<obj.pointsSub<<std::endl;
            //     std::cout<<"obj.pointsStem:\n"<<obj.pointsStem<<std::endl;
            //     std::cout<<"obj.pointsMain:\n"<<obj.pointsMain<<std::endl;
            //     std::cout<<"obj.timesSub:\n"<<obj.timesSub.transpose()<<std::endl;
            //     std::cout<<"obj.timesStem:\n"<<obj.timesStem.transpose()<<std::endl;
            //     std::cout<<"obj.timesMain:\n"<<obj.timesMain.transpose()<<std::endl;
            //     std::cout<<"obj.v:\n"<<v.transpose()<<std::endl;
            //     std::cout<<"obj.dy:\n"<<dy<<std::endl;
            //     std::cout<<"obj.vVel:\n"<<obj.vVel<<std::endl;
            //     std::cout<<"ceym:"<<ceym<<std::endl;
            //     std::cout<<"ceyst:"<<ceyst<<std::endl;
            //     std::cout<<"ceysu:"<<ceysu<<std::endl;
            //     std::cout<<"cep:"<<cep<<std::endl;
            //     std::cout<<"vi:"<<vi.transpose<<std::endl;
            //     std::cout<<"cost0:"<<cost<<std::endl;
            //     obj.c_++;
            // }

            double stemT = obj.timesStem.sum();
            attachPenaltyFunctional(obj.timesSub, obj.minco_br.Sub.getCoeffs(),
                                    obj.hPolyIdxSub, obj.hPolytopesSub, obj.hPolytopesCam,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.swarmTrajs_->seg_intersec_ids_sub_, obj.swarmTrajs_, stemT,
                                    cost, obj.partialGradByTimesSub, 
                                    obj.partialGradByCoeffsSub, false);
            // std::cout<<"cost1:"<<cost<<std::endl;

            attachPenaltyFunctional(obj.timesStem, obj.minco_br.Stem.getCoeffs(),
                                    obj.hPolyIdxStem, obj.hPolytopesStem, obj.hPolytopesCam,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.swarmTrajs_->seg_intersec_ids_stem_, obj.swarmTrajs_, 0.0,
                                    cost, obj.partialGradByTimesStem, 
                                    obj.partialGradByCoeffsStem, false);
            // std::cout<<"cost2:"<<cost<<std::endl;

            attachPenaltyFunctional(obj.timesMain, obj.minco_br.Main.getCoeffs(),
                                    obj.hPolyIdxMain, obj.hPolytopesMain, obj.hPolytopesCam,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.swarmTrajs_->seg_intersec_ids_main_, obj.swarmTrajs_, stemT,
                                    cost, obj.partialGradByTimesMain, 
                                    obj.partialGradByCoeffsMain, true);
            // std::cout<<"cost3:"<<cost<<std::endl;


            // obj.minco_br.propogateGrad(obj.partialGradByCoeffsStem, obj.partialGradByTimesStem,
            //                         obj.partialGradByCoeffsMain, obj.partialGradByTimesMain,
            //                         obj.partialGradByCoeffsSub, obj.partialGradByTimesSub,
            //                         obj.gradByStemVel, obj.gradByStemAcc,
            //                         obj.gradByPointsStem, obj.gradByTimesStem,
            //                         obj.gradByPointsMain, obj.gradByTimesMain,
            //                         obj.gradByPointsSub, obj.gradByTimesSub);

            obj.minco_br.propogateGrad(obj.partialGradByCoeffsStem, obj.partialGradByTimesStem,
                obj.partialGradByCoeffsMain, obj.partialGradByTimesMain,
                obj.partialGradByCoeffsSub, obj.partialGradByTimesSub,
                obj.gradByStemVel, obj.gradByStemAcc,
                obj.gradByMainVel, obj.gradByMainAcc,
                obj.gradByPointsStem, obj.gradByTimesStem,
                obj.gradByPointsMain, obj.gradByTimesMain,
                obj.gradByPointsSub, obj.gradByTimesSub);
            gradVsi = obj.gradByStemVel;
            gradAsi = obj.gradByStemAcc;
            gradVmi = obj.gradByMainVel;
            gradAmi = obj.gradByMainAcc;
            // std::cout<<"cost4:"<<cost<<std::endl;

            cost += weightT * (obj.timesMain.sum() + obj.timesStem.sum());
            obj.gradByTimesMain.array() += weightT;
            obj.gradByTimesStem.array() += weightT;

            backwardGradT(tauStem, obj.gradByTimesStem, gradTauStem);
            backwardGradT(tauMain, obj.gradByTimesMain, gradTauMain);
            backwardGradT(tauSub, obj.gradByTimesSub, gradTauSub);

            backwardGradP(xiStem, obj.vPolyIdxStem, obj.vPolytopesStem, yiStem, obj.yawRangeStem, obj.gradByPointsStem, gradXiStem, gradYiStem);
            backwardGradP(xiMain, obj.vPolyIdxMain, obj.vPolytopesMain, yiMain, obj.yawRangeMain, obj.gradByPointsMain, gradXiMain, gradYiMain);
            backwardGradP(xiSub, obj.vPolyIdxSub, obj.vPolytopesSub, yiSub, obj.yawRangeSub, obj.gradByPointsSub, gradXiSub, gradYiSub);


            // backwardGradVA(obj.gradByStemVel, vi, obj.vVel, dyi, obj.Dy, gradVi, gradDyi);
            // backwardGradVA(obj.gradByStemAcc, ai, obj.vAcc, ddyi, obj.Ddy, gradAi, gradDdyi);

            // backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
            // backwardGradV(evi, obj.vEndVel, obj.gradByEndVel, gradEvi);

            normRetrictionLayer(xiStem, obj.vPolyIdxStem, obj.vPolytopesStem, yiStem, cost, gradXiStem, gradYiStem);
            // std::cout<<"cost5:"<<cost<<std::endl;

            normRetrictionLayer(xiMain, obj.vPolyIdxMain, obj.vPolytopesMain, yiMain, cost, gradXiMain, gradYiMain);
            // std::cout<<"cost6:"<<cost<<std::endl;
            
            normRetrictionLayer(xiSub, obj.vPolyIdxSub, obj.vPolytopesSub, yiSub, cost, gradXiSub, gradYiSub);
            // std::cout<<"cost7:"<<cost<<std::endl;
            
            // normRetrictionLayerVA(vi, dyi, cost, gradVi, gradDyi);
            // normRetrictionLayerVA(ai, ddyi, cost, gradAi, gradDdyi);


            // std::cout<<"cost8:"<<cost<<std::endl;

            // normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);
            // normRetrictionLayerV(evi, cost, gradEvi);

            return cost;
        }

        /**
         * @brief interface of BFGS, shortten the path
         * 
         * @param ptr 
         * @param xi 
         * @param gradXi 
         * @return objective cost 
         */
        static inline double costDistance(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            void **dataPtrs = (void **)ptr;
            const double &dEps = *((const double *)(dataPtrs[0]));
            const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
            const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

            double cost = 0.0;
            const int overlaps = vPolys.size() / 2;

            Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
            Eigen::Vector3d a, b, d;
            Eigen::VectorXd r;
            double smoothedDistance;
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                a = i == 0 ? ini : b;
                if (i < overlaps)
                {
                    k = vPolys[2 * i + 1].cols();
                    Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
                }
                else
                {
                    b = fin;
                }

                d = b - a;
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;

                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;
                }
            }

            Eigen::VectorXd unitQ;
            double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
                sqrNormQ = q.squaredNorm();
                invNormQ = 1.0 / sqrt(sqrNormQ);
                unitQ = q * invNormQ;
                gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;


                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradQ += dc * 2.0 * q;
                }
            }

            return cost;
        }

        /**
         * @brief shortten the path
         * 
         * @param ini 
         * @param fin 
         * @param vPolys 
         * @param smoothD 
         * @param path 
         */
        static inline void getShortestPath(const Eigen::Vector3d &ini,
                                           const Eigen::Vector3d &fin,
                                           const PolyhedraV &vPolys,
                                           const double &smoothD,
                                           Eigen::Matrix3Xd &path)
        {
            const int overlaps = vPolys.size() / 2;
            Eigen::VectorXi vSizes(overlaps);
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();
            }
            Eigen::VectorXd xi(vSizes.sum());
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
                j += vSizes(i);
            }

            double minDistance;
            void *dataPtrs[4];
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);
            lbfgs::lbfgs_parameter_t shortest_path_params;
            shortest_path_params.past = 3;
            shortest_path_params.delta = 1.0e-3;
            shortest_path_params.g_epsilon = 1.0e-5;

            lbfgs::lbfgs_optimize(xi,
                                  minDistance,
                                  &GCOPTER_AGGRESSIVE::costDistance,
                                  nullptr,
                                  nullptr,
                                  dataPtrs,
                                  shortest_path_params);

            path.resize(3, overlaps + 2);
            path.leftCols<1>() = ini;
            path.rightCols<1>() = fin;
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                                  vPolys[2 * i + 1].col(0);
            }

            return;
        }

        /**
         * @brief get origin and vectors of each corridor. 
         * for quick hull corridors, not used here.
         * 
         * @param hPs ieqs of polytopes
         * @param vPs corridori: [origin, dir1, dir2....]
         * @return true 
         * @return false 
         */
        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV &vPs)
        {
            const int sizeCorridor = hPs.size() - 1;

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);

            int nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            for (int i = 0; i < sizeCorridor; i++)
            {
                if (!geo_utils::enumerateVs(hPs[i], curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);

                curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
                curIH.topRows(hPs[i].rows()) = hPs[i];
                curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
                if (!geo_utils::enumerateVs(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);
            }

            if (!geo_utils::enumerateVs(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            return true;
        }

        /**
         * @brief set the initial params 
         * 
         * @param path          initial path
         * @param speed         for init T
         * @param intervalNs    index of corresponding polytops
         * @param innerPoints   
         * @param timeAlloc 
         */
        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXd yawPath,
                                      const double &yawSpeed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix4Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {
            const int sizeM = intervalNs.size();
            const int sizeN = intervalNs.sum();
            innerPoints.resize(4, sizeN - 1);
            timeAlloc.resize(sizeN);

            Eigen::Vector4d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);
                a(3) = yawPath(i);
                b(3) = yawPath(i + 1);
                a.head(3) = path.col(i);
                b.head(3) = path.col(i + 1);
                c = (b - a) / l;
                // timeAlloc.segment(j, l).setConstant(c.norm() / speed);
                timeAlloc.segment(j, l).setConstant(std::max(c.head(3).norm() / speed + 1e-3, abs(c(3)) / yawSpeed + 1e-3));
                j += l;
                for (int m = 0; m < l; m++)
                {
                    if (i > 0 || m > 0)
                    {
                        innerPoints.col(k++) = a + c * m;
                    }
                }
            }
        }

    public:

        /**
         * @brief set path, corridors and optimization params
         * 
         */
        inline bool setup(const double &timeWeight,
                          const Eigen::Matrix4Xd &initialPVA,
                        //   const Eigen::Matrix3d &terminalPVA,
                          const Eigen::Vector4d &stemP,
                          const Eigen::Vector4d &mainP,
                          const Eigen::Vector4d &subP,
                          const PolyhedraH &corridorStem,
                          const PolyhedraV &CorridorStemV,
                          const PolyhedraH &corridorMain,
                          const PolyhedraV &CorridorMainV,
                          const PolyhedraH &corridorSub,
                          const PolyhedraV &CorridorSubV,
                          const PolyhedronH &CamFov,
                          const PolyhedronV &CamV,
                          const double &lengthPerPiece,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          SWARM_TRAJs *swarm_trajs)
                        //   const Eigen::VectorXd &physicalParams)
        {

            swarmTrajs_ = swarm_trajs;
            rho = timeWeight;
            headPVA = initialPVA;

            mainPVA.resize(4, 3);
            mainPVA.setZero();
            subPVA.resize(4, 3);
            subPVA.setZero();
            stemPVA.resize(4, 3);
            stemPVA.setZero();
            mainPVA.col(0) = mainP;
            subPVA.col(0) = subP;
            stemPVA.col(0) = stemP;
            hPolytopesCam = CamFov;

            vPolytopesStem = CorridorStemV;
            hPolytopesStem = corridorStem;
            for (size_t i = 0; i < hPolytopesStem.size(); i++)
            {
                const Eigen::ArrayXd norms =
                hPolytopesStem[i].leftCols<3>().rowwise().norm();
                hPolytopesStem[i].array().colwise() /= norms;
            }

            vPolytopesMain = CorridorMainV;
            hPolytopesMain = corridorMain;
            for (size_t i = 0; i < hPolytopesMain.size(); i++)
            {
                const Eigen::ArrayXd norms =
                hPolytopesMain[i].leftCols<3>().rowwise().norm();
                hPolytopesMain[i].array().colwise() /= norms;
            }

            vPolytopesSub = CorridorSubV;
            hPolytopesSub = corridorSub;
            for (size_t i = 0; i < hPolytopesSub.size(); i++)
            {
                const Eigen::ArrayXd norms =
                hPolytopesSub[i].leftCols<3>().rowwise().norm();
                hPolytopesSub[i].array().colwise() /= norms;
            }

            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            // physicalPm = physicalParams;
            allocSpeed = magnitudeBd(0) * 0.5;
            allocYawSpeed = magnitudeBd(3) * 0.5;
            // sub init
            int subPolyN = hPolytopesSub.size();
            getShortestPath(stemPVA.col(0).head(3), subPVA.col(0).head(3),
                            vPolytopesSub, smoothEps, shortPathSub);
            const Eigen::Matrix3Xd deltasSub = shortPathSub.rightCols(subPolyN) - shortPathSub.leftCols(subPolyN);
            pieceIdxSub = (deltasSub.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            const Eigen::VectorXd lengthsSub = deltasSub.colwise().norm().transpose();
            double total_l_sub = deltasSub.colwise().norm().sum();

            pieceIdxSub.array() += 1;
            pieceNSub = pieceIdxSub.sum();

            temporalDimSub = pieceNSub;
            spatialDimSub = 0;
            yawDimSub = 0;
            vPolyIdxSub.resize(pieceNSub - 1);
            hPolyIdxSub.resize(pieceNSub);

            yawPathSub.resize(pieceNSub + 1);
            yawPathSub(0) = stemPVA(3, 0);
            yawRangeSub.resize(pieceNSub - 1);
            double dyaw = subPVA(3, 0) - yawPathSub(0);
            double ll = 0;

            for (int i = 0, j = 0, k; i < subPolyN; i++)
            {
                k = pieceIdxSub(i);
                for (int l = 0; l < k; l++, j++)
                {
                    yawPathSub(j + 1) = dyaw * ((l + 1.0) / k * lengthsSub(i) + ll) / total_l_sub + yawPathSub(0);

                    if (l < k - 1)
                    {
                        yawRangeSub[j].first = yawPathSub(j + 1) - 1.5;
                        yawRangeSub[j].second = 3.0;
                        yawDimSub += 2;
                        vPolyIdxSub(j) = 2 * i;
                        spatialDimSub += vPolytopesSub[2 * i].cols();
                    }
                    else if (i < subPolyN - 1)
                    {
                        yawRangeSub[j].first = yawPathSub(j + 1) - 1.5;
                        yawRangeSub[j].second = 3.0;
                        yawDimSub += 2;
                        vPolyIdxSub(j) = 2 * i + 1;
                        spatialDimSub += vPolytopesSub[2 * i + 1].cols();
                    }
                    hPolyIdxSub(j) = i;
                }
                ll += lengthsSub(i);
            }

            // stem init
            int stemPolyN = hPolytopesStem.size();
            getShortestPath(headPVA.col(0).head(3), stemPVA.col(0).head(3),
                            vPolytopesStem, smoothEps, shortPathStem);
            const Eigen::Matrix3Xd deltasStem = shortPathStem.rightCols(stemPolyN) - shortPathStem.leftCols(stemPolyN);
            pieceIdxStem = (deltasStem.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            const Eigen::VectorXd lengthsStem = deltasStem.colwise().norm().transpose();
            double total_l_stem = deltasStem.colwise().norm().sum();

            pieceIdxStem.array() += 1;
            pieceNStem = pieceIdxStem.sum();

            temporalDimStem = pieceNStem;
            spatialDimStem = 0;
            yawDimStem = 0;
            vPolyIdxStem.resize(pieceNStem - 1);
            hPolyIdxStem.resize(pieceNStem);

            yawPathStem.resize(pieceNStem + 1);
            yawPathStem(0) = headPVA(3, 0);
            yawRangeStem.resize(pieceNStem - 1);
            dyaw = stemPVA(3, 0) - yawPathStem(0);
            ll = 0;

            for (int i = 0, j = 0, k; i < stemPolyN; i++)
            {
                k = pieceIdxStem(i);
                for (int l = 0; l < k; l++, j++)
                {
                    yawPathStem(j + 1) = dyaw * ((l + 1.0) / k * lengthsStem(i) + ll) / total_l_stem + yawPathStem(0);
                    if (l < k - 1)
                    {
                        yawRangeStem[j].first = yawPathStem(j + 1) - 1.5;
                        yawRangeStem[j].second = 3.0;
                        yawDimStem += 2;
                        vPolyIdxStem(j) = 2 * i;
                        spatialDimStem += vPolytopesStem[2 * i].cols();
                        // std::cout<<"spatialDimStem1:"<<spatialDimStem<<std::endl;
                    }
                    else if (i < stemPolyN - 1)
                    {
                        yawRangeStem[j].first = yawPathStem(j + 1) - 1.5;
                        yawRangeStem[j].second = 3.0;
                        yawDimStem += 2;
                        vPolyIdxStem(j) = 2 * i + 1;
                        spatialDimStem += vPolytopesStem[2 * i + 1].cols();
                        // std::cout<<"spatialDimStem2:"<<spatialDimStem<<std::endl;
                    }
                    hPolyIdxStem(j) = i;
                }
                ll += lengthsStem(i);
            }

            // main init
            int mainPolyN = hPolytopesMain.size();
            getShortestPath(stemPVA.col(0).head(3), mainPVA.col(0).head(3),
                                vPolytopesMain, smoothEps, shortPathMain);
            const Eigen::Matrix3Xd deltasMain = shortPathMain.rightCols(mainPolyN) - shortPathMain.leftCols(mainPolyN);
            pieceIdxMain = (deltasMain.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            const Eigen::VectorXd lengthsMain = deltasMain.colwise().norm().transpose();
            double total_l_main = deltasMain.colwise().norm().sum();

            pieceIdxMain.array() += 1;
            pieceNMain = pieceIdxMain.sum();

            temporalDimMain = pieceNMain;
            spatialDimMain = 0;
            yawDimMain = 0;
            vPolyIdxMain.resize(pieceNMain - 1);
            hPolyIdxMain.resize(pieceNMain);

            yawPathMain.resize(pieceNMain + 1);
            yawPathMain(0) = stemPVA(3, 0);
            yawRangeMain.resize(pieceNMain - 1);
            dyaw = mainPVA(3, 0) - yawPathMain(0);
            ll = 0;

            for (int i = 0, j = 0, k; i < mainPolyN; i++)
            {
                k = pieceIdxMain(i);
                for (int l = 0; l < k; l++, j++)
                {
                    yawPathMain(j + 1) = dyaw * ((l + 1.0) / k * lengthsMain(i) + ll) / total_l_main + yawPathMain(0);
                    if (l < k - 1)
                    {
                        yawRangeMain[j].first = yawPathMain(j + 1) - 1.5;
                        yawRangeMain[j].second = 3.0;
                        yawDimMain += 2;
                        vPolyIdxMain(j) = 2 * i;
                        spatialDimMain += vPolytopesMain[2 * i].cols();
                    }
                    else if (i < mainPolyN - 1)
                    {
                        yawRangeMain[j].first = yawPathMain(j + 1) - 1.5;
                        yawRangeMain[j].second = 3.0;
                        yawDimMain += 2;
                        vPolyIdxMain(j) = 2 * i + 1;
                        spatialDimMain += vPolytopesMain[2 * i + 1].cols();
                    }
                    hPolyIdxMain(j) = i;
                }
                ll += lengthsMain(i);
            }

            // VelDim = 0;
            // PolyhedronH velh, curIH;
            // velh.resize(6, 4);
            // velh.setZero();
            // for(int i = 0; i < 3; i++){
            //     velh(i*2, i) = 1.0;
            //     velh(i*2, 3) = -magnitudeBounds[0] * 0.95;
            //     velh(i*2+1, i) = -1.0;
            //     velh(i*2+1, 3) = -magnitudeBounds[0] * 0.95;
            // }
            // curIH.resize(6 + hPolytopesCam.rows(), 4);
            // curIH.topRows(6) = velh;
            // curIH.bottomRows(hPolytopesCam.rows()) = hPolytopesCam;
            // Eigen::Matrix3d R;
            // R << cos(stemP(3)), -sin(stemP(3)), 0, sin(stemP(3)), cos(stemP(3)), 0, 0, 0, 1;
            // curIH.block(6, 0, hPolytopesCam.rows(), 3) = curIH.block(6, 0, hPolytopesCam.rows(), 3) * R.transpose();

            // bool useRawCorridor = geo_utils::enumerateVs(curIH, vVel);
            // bool useRawCorridor = false;
            // if(useRawCorridor){
            //     VelDim = vVel.cols();
            //     for(int i = 0; i < vVel.cols(); i++){
            //         vVel.col(i) = vVel.col(i) - vVel.col(0);
            //     }
            // }
            // else{
            // PosDim = 8;
            // vPos.resize(3, 8);
            // Eigen::Matrix<double, 3, 2> bound;
            // Eigen::Vector3d pmax, pmin; 
            // for(int i = 0; i < 3; i++){
            //     vPolytopesStem.back().row(i).maxCoeff();
            //     // pmax(i) = min(stemP(i) + 0.35, corridorStemV.back().)
            //     pmin(i) = stemP(i) - 0.35;
            // }
            // for(int dim1 = 0; dim1 <= 1; dim1++){
            //     for(int dim2 = 0; dim2 <= 1; dim2++){
            //         for(int dim3 = 0; dim3 <= 1; dim3++){
            //             vPos(0, 4*dim3 + 2*dim2 + dim1) = dim1 ? pmax(0) : pmin(0);
            //             vPos(1, 4*dim3 + 2*dim2 + dim1) = dim2 ? pmax(1) : pmin(1);
            //             vPos(2, 4*dim3 + 2*dim2 + dim1) = dim3 ? pmax(2) : pmin(2);
            //         }
            //     }
            // }
            // for(int j = 1; j < 8; j++){
            //     vPos.col(j) = vPos.col(j) - vPos.col(0);
            // }


            // VelDim = 8;
            // vVel.resize(3, 8);
            // Eigen::Matrix<double, 3, 2> bound;
            // Eigen::Vector3d vmax, vmin; 
            // for(int i = 0; i < 3; i++){
            //     vmax(i) = magnitudeBounds[0] * 10.0;
            //     vmin(i) = -magnitudeBounds[0] * 10.0;
            // }
            // for(int dim1 = 0; dim1 <= 1; dim1++){
            //     for(int dim2 = 0; dim2 <= 1; dim2++){
            //         for(int dim3 = 0; dim3 <= 1; dim3++){
            //             vVel(0, 4*dim3 + 2*dim2 + dim1) = dim1 ? vmax(0) : vmin(0);
            //             vVel(1, 4*dim3 + 2*dim2 + dim1) = dim2 ? vmax(1) : vmin(1);
            //             vVel(2, 4*dim3 + 2*dim2 + dim1) = dim3 ? vmax(2) : vmin(2);
            //         }
            //     }
            // }
            // for(int j = 1; j < 8; j++){
            //     vVel.col(j) = vVel.col(j) - vVel.col(0);
            // }

            // AccDim = 8;
            // vAcc.resize(3, 8);
            // Eigen::Vector3d amax, amin; 
            // for(int i = 0; i < 3; i++){
            //     amax(i) = magnitudeBounds[1] * 10.0;
            //     amin(i) = -magnitudeBounds[1] * 10.0;
            // }
            // for(int dim1 = 0; dim1 <= 1; dim1++){
            //     for(int dim2 = 0; dim2 <= 1; dim2++){
            //         for(int dim3 = 0; dim3 <= 1; dim3++){
            //             vAcc(0, 4*dim3 + 2*dim2 + dim1) = dim1 ? amax(0) : amin(0);
            //             vAcc(1, 4*dim3 + 2*dim2 + dim1) = dim2 ? amax(1) : amin(1);
            //             vAcc(2, 4*dim3 + 2*dim2 + dim1) = dim3 ? amax(2) : amin(2);
            //         }
            //     }
            // }
            // for(int j = 1; j < 8; j++){
            //     vAcc.col(j) = vAcc.col(j) - vAcc.col(0);
            // }




            // }
            // vVel = R * CamV;
            // VelDim = vVel.cols();
            
            // std::cout<<"temporalDimSub:"<<temporalDimSub<<std::endl;
            // std::cout<<"temporalDimStem:"<<temporalDimStem<<std::endl;
            // std::cout<<"temporalDimMain:"<<temporalDimMain<<std::endl;
            // std::cout<<"spatialDimSub:"<<spatialDimSub<<std::endl;
            // std::cout<<"spatialDimStem:"<<spatialDimStem<<std::endl;
            // std::cout<<"spatialDimMain:"<<spatialDimMain<<std::endl;
            // std::cout<<"yawDimSub:"<<yawDimSub<<std::endl;
            // std::cout<<"yawDimStem:"<<yawDimStem<<std::endl;
            // std::cout<<"yawDimMain:"<<yawDimMain<<std::endl;
            // std::cout<<"VelDim:"<<VelDim<<std::endl;
            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco_br.setPieces(pieceNStem, pieceNMain, pieceNSub);
            minco_br.setConditions(headPVA, mainPVA, subPVA);
            hPolytopesCam.col(3).array() += 0.0;

            // Allocate temp variables
            pointsSub.resize(4, pieceNSub - 1);
            pointsMain.resize(4, pieceNMain - 1);
            pointsStem.resize(4, pieceNStem - 1);
            timesSub.resize(pieceNSub);
            timesMain.resize(pieceNMain);
            timesStem.resize(pieceNStem);
            gradByPointsSub.resize(4, pieceNSub - 1);
            gradByPointsStem.resize(4, pieceNStem - 1);
            gradByPointsMain.resize(4, pieceNMain - 1);
            gradByTimesSub.resize(pieceNSub);
            gradByTimesStem.resize(pieceNStem);
            gradByTimesMain.resize(pieceNMain);
            partialGradByCoeffsSub.resize(6 * pieceNSub, 4);
            partialGradByCoeffsStem.resize(6 * pieceNStem, 4);
            partialGradByCoeffsMain.resize(6 * pieceNMain, 4);
            partialGradByTimesSub.resize(pieceNSub);
            partialGradByTimesStem.resize(pieceNStem);
            partialGradByTimesMain.resize(pieceNMain);
            Dy.resize(2);
            Dy(0) = -magnitudeBounds(3);
            Dy(1) = 2.0*magnitudeBounds(3);
            Ddy.resize(2);
            Ddy(0) = -magnitudeBounds(4);
            Ddy(1) = 2.0*magnitudeBounds(4);
            return true;
        }


        bool CollideCheck(const Eigen::VectorXd &T,
            const Eigen::MatrixX4d &coeffs,
            const Eigen::VectorXi &hIdx,
            const PolyhedraH &hPolys,
            const double &smoothFactor,
            const int &integralResolution,
            const Eigen::VectorXd &magnitudeBounds,
            const Eigen::VectorXd &penaltyWeights,
            const std::vector<std::vector<int>> &seg_intersec_ids,
            SWARM_TRAJs *swarmTrajs,
            const double &t0){
            const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
            const double accSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
            const double dyawMax = magnitudeBounds(3) * magnitudeBounds(3);
            const double ddyawMax = magnitudeBounds(4) * magnitudeBounds(4);
            const double swarmSqrMax = magnitudeBounds(5) * magnitudeBounds(5);

            // const double weightPos = penaltyWeights(0);
            // const double weightVel = penaltyWeights(1);
            // const double weightAcc = penaltyWeights(2);
            // const double weightJer = penaltyWeights(3);
            // const double weightDyaw = penaltyWeights(4);
            // const double weightDdyaw = penaltyWeights(5);
            // const double weightSwarm = penaltyWeights(6);

            Eigen::Vector4d pos, vel, acc, jer, sna;
            // Eigen::Vector4d gradPos, gradVel, gradAcc, gradJer;
            Eigen::Vector3d pSwarm, vSwarm, dps;

            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;
            double step, tse, tc, tsi;


            double s1, s2, s3, s4, s5;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d outerNormal;
            int S, K, L;
            double violaSwarm;//, violaVc;
            double violaSwarmPena;//, violaVcPena;
            double violaSwarmPenaD;//, violaVcPenaD;
            double violaPos;
            double violaPosPena;
            double violaPosPenaD;
            // D = swarmTrajs->trajs_.size();
            S = swarmTrajs->SwarmPos_.size();
            tse = 0.0;
            for (int i = 0; i < pieceNum; i++)
            {
                L = hIdx(i);
                K = hPolys[L].rows();
                const Eigen::Matrix<double, 6, 4> &c = coeffs.block<6, 4>(i * 6, 0);
                step = T(i) * integralFrac;
                for (int j = 0; j <= integralResolution; j++)
                {

                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;
                    tc = tse + j * step + t0;

                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = hPolys[L].block<1, 3>(k, 0);
                        violaPos = outerNormal.dot(pos.head(3)) + hPolys[L](k, 3) - 0.5;
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            std::cout<<"viola pos:"<<pos.transpose()<<std::endl;
                            return false;
                        }
                    }

                    if(vel.head(3).squaredNorm() - velSqrMax > 1.0){
                        std::cout<<"viola vel:"<<vel.norm()<<std::endl;
                        return false;
                    }
                    if(acc.head(3).squaredNorm() - accSqrMax > 1.0){
                        std::cout<<"viola acc:"<<acc.norm()<<std::endl;
                        return false;
                    }
                    for(auto &id : seg_intersec_ids[i]){
                        // std::cout<<"id:"<<id<<std::endl;
                        // std::cout<<"swarmTrajs->Ts_:"<<swarmTrajs->Ts_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->trajs_:"<<swarmTrajs->trajs_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->nVs_:"<<swarmTrajs->nVs_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->durations_:"<<swarmTrajs->durations_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->Ve_:"<<swarmTrajs->Ve_.size()<<std::endl;
                        // std::cout<<"swarmTrajs->Pe_:"<<swarmTrajs->Pe_.size()<<std::endl;
                        tsi = tc - swarmTrajs->Ts_.at(id);
                        if(tsi < 0){
                            pSwarm = swarmTrajs->Ps_.at(id) + tsi * swarmTrajs->nVs_.at(id);
                            vSwarm = -swarmTrajs->nVs_.at(id);
                        }
                        else if(tsi < swarmTrajs->durations_.at(id)){
                            pSwarm = swarmTrajs->trajs_.at(id).getPos(tsi).head(3);
                            vSwarm = swarmTrajs->trajs_.at(id).getVel(tsi).head(3);
                        }
                        else{
                            pSwarm = swarmTrajs->Pe_.at(id) + (tsi - swarmTrajs->durations_.at(id)) * swarmTrajs->Ve_.at(id);
                            vSwarm = swarmTrajs->Ve_.at(id);
                        }
                        dps = pSwarm - pos.head(3);
                        violaSwarm = swarmSqrMax - (dps).squaredNorm();
                        if (smoothedL1(violaSwarm, smoothFactor, violaSwarmPena, violaSwarmPenaD))
                        {
                            std::cout<<"pos:"<<pos.transpose()<<std::endl;
                            std::cout<<"dps:"<<dps.norm()<<std::endl;
                            std::cout<<"pSwarm:"<<pSwarm.transpose()<<std::endl;
                        }
                    }

                    // swarm pos cost
                    for(int id = 0; id < S; id++){
                        dps = swarmTrajs->SwarmPos_.at(id) - pos.head(3);
                        violaSwarm = swarmSqrMax - (dps).squaredNorm();
                        if (smoothedL1(violaSwarm, smoothFactor, violaSwarmPena, violaSwarmPenaD))
                        {
                            std::cout<<"pos:"<<pos.transpose()<<std::endl;
                            std::cout<<"swarmTrajs->SwarmPos_.at(id):"<<swarmTrajs->SwarmPos_.at(id).transpose()<<std::endl;
                        }
                    }
                }
            }
            return true;
        }

        bool CollideCheck(){
            if(!CollideCheck(timesStem, minco_br.Stem.getCoeffs(), hPolyIdxStem, hPolytopesStem, smoothEps, integralRes, magnitudeBd,
            penaltyWt, swarmTrajs_->seg_intersec_ids_stem_, swarmTrajs_, 0.0)){
                cout<<"stem collide"<<endl;
                return false;
            }
            double stemT = timesStem.sum();
            if(!CollideCheck(timesMain, minco_br.Main.getCoeffs(), hPolyIdxMain, hPolytopesMain, smoothEps, integralRes, magnitudeBd,
            penaltyWt, swarmTrajs_->seg_intersec_ids_main_, swarmTrajs_, stemT)){
                cout<<"main collide"<<endl;
                return false;
            }
            if(!CollideCheck(timesSub, minco_br.Sub.getCoeffs(), hPolyIdxSub, hPolytopesSub, smoothEps, integralRes, magnitudeBd,
            penaltyWt, swarmTrajs_->seg_intersec_ids_sub_, swarmTrajs_, stemT)){
                cout<<"sub collide"<<endl;
                return false;
            }
            return true;
        }

        /**
         * @brief call after setup()
         * 
         * @param trajStem       the optimized stem traj   
         * @param trajMain       the optimized main traj   
         * @param trajSub        the optimized sub traj
         * @param relCostTol 
         * @return double 
         */
        inline double optimize(Trajectory4<5> &trajStem,
                                Trajectory4<5> &trajMain,
                                Trajectory4<5> &trajSub,
                               const double &relCostTol)
        {

            // cout<<"opt0"<<endl;
            Eigen::VectorXd xBest;
            Eigen::VectorXd x(temporalDimSub + temporalDimStem + temporalDimMain + spatialDimSub + spatialDimStem + spatialDimMain + 
                                    yawDimSub + yawDimStem + yawDimMain + 4 + 4 + 4 + 4);
            int s = 0;
            Eigen::Map<Eigen::VectorXd> tauSub(x.data() + s, temporalDimSub);
            s += temporalDimSub;
            Eigen::Map<Eigen::VectorXd> tauStem(x.data() + s, temporalDimStem);
            s += temporalDimStem;
            Eigen::Map<Eigen::VectorXd> tauMain(x.data() + s, temporalDimMain);
            s += temporalDimMain;
            // cout<<"opt1"<<endl;

            Eigen::Map<Eigen::VectorXd> xiSub(x.data() + s, spatialDimSub);
            s += spatialDimSub;
            Eigen::Map<Eigen::VectorXd> xiStem(x.data() + s, spatialDimStem);
            s += spatialDimStem;
            Eigen::Map<Eigen::VectorXd> xiMain(x.data() + s, spatialDimMain);
            s += spatialDimMain;
            // cout<<"opt2"<<endl;

            Eigen::Map<Eigen::VectorXd> yiSub(x.data() + s, yawDimSub);
            s += yawDimSub;
            Eigen::Map<Eigen::VectorXd> yiStem(x.data() + s, yawDimStem);
            s += yawDimStem;
            Eigen::Map<Eigen::VectorXd> yiMain(x.data() + s, yawDimMain);
            s += yawDimMain;
            Eigen::Map<Eigen::VectorXd> vsi(x.data() + s, 4);
            s += 4;
            Eigen::Map<Eigen::VectorXd> asi(x.data() + s, 4);
            s += 4;
            Eigen::Map<Eigen::VectorXd> vmi(x.data() + s, 4);
            s += 4;
            Eigen::Map<Eigen::VectorXd> ami(x.data() + s, 4);

            // Eigen::Map<Eigen::VectorXd> vi(x.data() + s, VelDim);
            // s += VelDim;
            // Eigen::Map<Eigen::VectorXd> dyi(x.data() + s, 2);
            // s += 2;
            // Eigen::Map<Eigen::VectorXd> ai(x.data() + s, AccDim);
            // s += AccDim;
            // Eigen::Map<Eigen::VectorXd> ddyi(x.data() + s, 2);
            // cout<<"opt3"<<endl;

            // points are downsampled points of shortPath 
            setInitial(shortPathSub, allocSpeed, yawPathSub, allocYawSpeed, pieceIdxSub, pointsSub, timesSub);
            setInitial(shortPathStem, allocSpeed, yawPathStem, allocYawSpeed, pieceIdxStem, pointsStem, timesStem);
            setInitial(shortPathMain, allocSpeed, yawPathMain, allocYawSpeed, pieceIdxMain, pointsMain, timesMain);
            // cout<<"opt4"<<endl;
            timesStem.array() += 0.025 / timesStem.size();
            timesMain.array() += 0.025 / timesMain.size();
            timesSub.array() += 2.0;
            // setInitial(shortPathSub, allocSpeed, pieceIdxSub, pointsSub, timesSub);
            // setInitial(shortPathStem, allocSpeed, pieceIdxStem, pointsStem, timesStem);
            // setInitial(shortPathMain, allocSpeed, pieceIdxMain, pointsMain, timesMain);

            backwardT(timesSub, tauSub);
            backwardT(timesStem, tauStem);
            backwardT(timesMain, tauMain);
            backwardP(pointsSub, vPolyIdxSub, vPolytopesSub, xiSub);
            backwardP(pointsStem, vPolyIdxStem, vPolytopesStem, xiStem);
            backwardP(pointsMain, vPolyIdxMain, vPolytopesMain, xiMain);
            // cout<<"opt5"<<endl;

            // evi.setOnes();
            yiSub.setOnes();
            yiSub = yiSub / sqrt(yiSub.size() + 1e-3);
            yiStem.setOnes();
            yiStem = yiStem / sqrt(yiStem.size() + 1e-3);
            yiMain.setOnes();
            yiMain = yiMain / sqrt(yiMain.size() + 1e-3);
            // cout<<"opt6"<<endl;



            vsi.setZero();
            if((mainPVA.col(0).head(3) - stemPVA.col(0).head(3)).norm() > 1e-3)
                vsi.head(3) = (mainPVA.col(0).head(3) - stemPVA.col(0).head(3)).normalized() * magnitudeBd[0];
            asi.setZero();
            vmi.setZero();
            if((mainPVA.col(0).head(3) - stemPVA.col(0).head(3)).norm() > 1e-3)
                vmi.head(3) = (mainPVA.col(0).head(3) - stemPVA.col(0).head(3)).normalized() * magnitudeBd[0];
            ami.setZero();

            // dyi.setOnes();
            // dyi = dyi / sqrt(2.0);
            // vi.setOnes();
            // vi = vi / sqrt(vi.size() + 1e-3);
            // ddyi.setOnes();
            // ddyi = ddyi / sqrt(2.0);
            // ai.setOnes();
            // ai = ai / sqrt(ai.size() + 1e-3);
            // cout<<"opt7"<<endl;

            // Eigen::Vector3d v, a;
            // double dy, ddy;
            // forwardVA(vVel, vi, Dy, dyi, v, dy);
            // forwardVA(vAcc, ai, Ddy, ddyi, a, ddy);
            // cout<<"v:\n"<<v.transpose()<<endl;
            // cout<<"dy:\n"<<dy<<endl;
            // c_ = 0;
            double minCostFunctional;
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 40;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;
            // lbfgs_params.delta = 1.0e-2;
            // cout<<"opt8"<<endl;
            // cout<<"x0:"<<x.transpose()<<endl;
            int ret;
            double t_best = 999999.0;
            // xBest
            for(int i = 0; i < 5; i++){

                ret = lbfgs::lbfgs_optimize(x,
                                                minCostFunctional,
                                                &GCOPTER_AGGRESSIVE::costFunctional,
                                                nullptr,
                                                nullptr,
                                                this,
                                                lbfgs_params);

                forwardT(tauSub, timesSub);
                forwardT(tauStem, timesStem);
                forwardT(tauMain, timesMain);
                forwardP(xiSub, vPolyIdxSub, vPolytopesSub, yawRangeSub, yiSub, pointsSub);
                forwardP(xiStem, vPolyIdxStem, vPolytopesStem, yawRangeStem, yiStem, pointsStem);
                forwardP(xiMain, vPolyIdxMain, vPolytopesMain, yawRangeMain, yiMain, pointsMain);
                minco_br.setParameters(stemPVA, mainPVA, pointsStem, timesStem, pointsMain, timesMain, pointsSub, timesSub);
                // cout<<"i:"<<i<<" t:"<<timesStem.sum() + timesMain.sum()<<endl;
                // cout<<"CollideCheck:"<<CollideCheck()<<endl;
                if(i == 0){
                    if(CollideCheck()){
                        xBest = x;
                        t_best = timesStem.sum() + timesMain.sum();
                    }
                }
                else{
                    if(timesStem.sum() + timesMain.sum() < t_best && CollideCheck()){
                        xBest = x;
                        t_best = timesStem.sum() + timesMain.sum();
                    }
                }
                timesStem *= 0.8;
                timesMain *= 0.8;


                // if(vsi.head(3).norm() > 0.01){
                //     vsi.head(3) = vsi.head(3).normalized() * magnitudeBd[0];
                // }
                // if(vmi.head(3).norm() > 0.01){
                //     vmi.head(3) = vmi.head(3).normalized() * magnitudeBd[0];
                // }
                backwardT(timesSub, tauSub);
                backwardT(timesStem, tauStem);
                backwardT(timesMain, tauMain);
            }

            if(t_best < 999998.0){
                x = xBest;
                timesStem *= 0.8;
                timesMain *= 0.8;
                // ami.setZero();
                // asi.setZero();
            }
            lbfgs_params.max_iterations = 200;
            ret = lbfgs::lbfgs_optimize(x,
                minCostFunctional,
                &GCOPTER_AGGRESSIVE::costFunctional,
                nullptr,
                nullptr,
                this,
                lbfgs_params);
            forwardT(tauSub, timesSub);
            forwardT(tauStem, timesStem);
            forwardT(tauMain, timesMain);
            forwardP(xiSub, vPolyIdxSub, vPolytopesSub, yawRangeSub, yiSub, pointsSub);
            forwardP(xiStem, vPolyIdxStem, vPolytopesStem, yawRangeStem, yiStem, pointsStem);
            forwardP(xiMain, vPolyIdxMain, vPolytopesMain, yawRangeMain, yiMain, pointsMain);
            minco_br.setParameters(stemPVA, mainPVA, pointsStem, timesStem, pointsMain, timesMain, pointsSub, timesSub);
            // cout<<"traj last t:"<<timesStem.sum() + timesMain.sum()<<endl;
            // cout<<"CollideCheck:"<<CollideCheck()<<endl;
            if(timesStem.sum() + timesMain.sum() < t_best && CollideCheck()){
                xBest = x;
                t_best = timesStem.sum() + timesMain.sum();
            }
            // cout<<"opt9"<<endl;

            if (t_best < 999998.0)
            {


                s = 0;
                Eigen::Map<Eigen::VectorXd> tauSubBest(xBest.data() + s, temporalDimSub);
                s += temporalDimSub;
                Eigen::Map<Eigen::VectorXd> tauStemBest(xBest.data() + s, temporalDimStem);
                s += temporalDimStem;
                Eigen::Map<Eigen::VectorXd> tauMainBest(xBest.data() + s, temporalDimMain);
                s += temporalDimMain;
                // cout<<"opt1"<<endl;
    
                Eigen::Map<Eigen::VectorXd> xiSubBest(xBest.data() + s, spatialDimSub);
                s += spatialDimSub;
                Eigen::Map<Eigen::VectorXd> xiStemBest(xBest.data() + s, spatialDimStem);
                s += spatialDimStem;
                Eigen::Map<Eigen::VectorXd> xiMainBest(xBest.data() + s, spatialDimMain);
                s += spatialDimMain;
                // cout<<"opt2"<<endl;
    
                Eigen::Map<Eigen::VectorXd> yiSubBest(xBest.data() + s, yawDimSub);
                s += yawDimSub;
                Eigen::Map<Eigen::VectorXd> yiStemBest(xBest.data() + s, yawDimStem);
                s += yawDimStem;
                Eigen::Map<Eigen::VectorXd> yiMainBest(xBest.data() + s, yawDimMain);
                s += yawDimMain;
                Eigen::Map<Eigen::VectorXd> vsiBest(xBest.data() + s, 4);
                s += 4;
                Eigen::Map<Eigen::VectorXd> asiBest(xBest.data() + s, 4);
                s += 4;
                Eigen::Map<Eigen::VectorXd> vmiBest(xBest.data() + s, 4);
                s += 4;
                Eigen::Map<Eigen::VectorXd> amiBest(xBest.data() + s, 4);

                forwardT(tauSubBest, timesSub);
                forwardT(tauStemBest, timesStem);
                forwardT(tauMainBest, timesMain);
                forwardP(xiSubBest, vPolyIdxSub, vPolytopesSub, yawRangeSub, yiSubBest, pointsSub);
                forwardP(xiStemBest, vPolyIdxStem, vPolytopesStem, yawRangeStem, yiStemBest, pointsStem);
                forwardP(xiMainBest, vPolyIdxMain, vPolytopesMain, yawRangeMain, yiMainBest, pointsMain);
                // Eigen::Vector3d v;
                // double dy;
                // forwardVA(vVel, vi, Dy, dyi, v, dy);
                // forwardVA(vAcc, ai, Ddy, ddyi, a, ddy);

                // Eigen::VectorXd tMs(pieceNStem + pieceNMain);
                // tMs.head(pieceNStem) = timesStem;
                // tMs.head(pieceNMain) = timesMain;
                // Eigen::MatrixXd ptsMs(4, pieceNMain + pieceNStem - 1);
                // ptsMs.leftCols(pieceNStem - 1) = pointsStem;
                // ptsMs.col(pieceNStem - 1) = stemPVA.col(0);
                // ptsMs.rightCols(pieceNMain - 1) = pointsMain;


                // stemPVA.col(1).head(3) = v;
                // stemPVA(3, 1) = dy;
                // stemPVA.col(2).head(3) = a;
                // stemPVA(3, 2) = ddy;

                stemPVA.col(1) = vsiBest;
                stemPVA.col(2) = asiBest;
                mainPVA.col(1) = vmiBest;
                mainPVA.col(2) = amiBest;

                // minco_br.setParameters(stemPVA, pointsStem, timesStem, pointsMain, timesMain, pointsSub, timesSub);
                minco_br.setParameters(stemPVA, mainPVA, pointsStem, timesStem, pointsMain, timesMain, pointsSub, timesSub);

                minco_br.Stem.getTrajectory(trajStem);
                minco_br.Main.getTrajectory(trajMain);
                minco_br.Sub.getTrajectory(trajSub);
                cout<<"stemPVA:\n"<<stemPVA<<endl;
                cout<<"mainPVA:\n"<<mainPVA<<endl;
                cout<<"t_best:"<<t_best<<endl;
                // double stemT = timesStem.sum();
                // if(!CollideCheck(timesStem, minco_br.Stem.getCoeffs(), hPolyIdxStem, hPolytopesStem, smoothEps, integralRes, magnitudeBd,
                // penaltyWt, swarmTrajs_->seg_intersec_ids_stem_, swarmTrajs_, 0.0)){
                //     cout<<"stem collide"<<endl;
                //     minCostFunctional = INFINITY;
                // }
                // if(!CollideCheck(timesMain, minco_br.Main.getCoeffs(), hPolyIdxMain, hPolytopesMain, smoothEps, integralRes, magnitudeBd,
                // penaltyWt, swarmTrajs_->seg_intersec_ids_main_, swarmTrajs_, stemT)){
                //     cout<<"main collide"<<endl;
                //     minCostFunctional = INFINITY;
                // }
                // if(!CollideCheck(timesSub, minco_br.Sub.getCoeffs(), hPolyIdxSub, hPolytopesSub, smoothEps, integralRes, magnitudeBd,
                // penaltyWt, swarmTrajs_->seg_intersec_ids_sub_, swarmTrajs_, stemT)){
                //     cout<<"sub collide"<<endl;
                //     minCostFunctional = INFINITY;
                // }
                // minco_br.setParameters(ptsMs, tMs, pointsSub, timesSub);
                // minco_br.SteMa.getTrajectory(trajMs);
                // minco_br.Sub.getTrajectory(trajSub);
            }
            // else if(ret == lbfgs::LBFGSERR_MAXIMUMITERATION || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
            //     forwardT(tauSub, timesSub);
            //     forwardT(tauStem, timesStem);
            //     forwardT(tauMain, timesMain);
            //     forwardP(xiSub, vPolyIdxSub, vPolytopesSub, yawRangeSub, yiSub, pointsSub);
            //     forwardP(xiStem, vPolyIdxStem, vPolytopesStem, yawRangeStem, yiStem, pointsStem);
            //     forwardP(xiMain, vPolyIdxMain, vPolytopesMain, yawRangeMain, yiMain, pointsMain);
            //     // Eigen::Vector3d v;
            //     // double dy;
            //     // forwardVA(vVel, vi, Dy, dyi, v, dy);
            //     // forwardVA(vAcc, ai, Ddy, ddyi, a, ddy);

            //     // stemPVA.col(1).head(3) = v;
            //     // stemPVA(3, 1) = dy;
            //     // stemPVA.col(2).head(3) = a;
            //     // stemPVA(3, 2) = ddy;

            //     stemPVA.col(1) = vsi;
            //     stemPVA.col(2) = asi;
            //     mainPVA.col(1) = vmi;
            //     mainPVA.col(2) = ami;

            //     // minco_br.setParameters(stemPVA, pointsStem, timesStem, pointsMain, timesMain, pointsSub, timesSub);
            //     minco_br.setParameters(stemPVA, mainPVA, pointsStem, timesStem, pointsMain, timesMain, pointsSub, timesSub);
            //     minco_br.Stem.getTrajectory(trajStem);
            //     minco_br.Main.getTrajectory(trajMain);
            //     minco_br.Sub.getTrajectory(trajSub);

            //     double stemT = timesStem.sum();
            //     if(!CollideCheck(timesStem, minco_br.Stem.getCoeffs(), hPolyIdxStem, hPolytopesStem, smoothEps, integralRes, magnitudeBd,
            //     penaltyWt, swarmTrajs_->seg_intersec_ids_stem_, swarmTrajs_, 0.0)){
            //         cout<<"stem collide"<<endl;
            //         minCostFunctional = INFINITY;
            //     }
            //     if(!CollideCheck(timesMain, minco_br.Main.getCoeffs(), hPolyIdxMain, hPolytopesMain, smoothEps, integralRes, magnitudeBd,
            //     penaltyWt, swarmTrajs_->seg_intersec_ids_main_, swarmTrajs_, stemT)){
            //         cout<<"main collide"<<endl;
            //         minCostFunctional = INFINITY;
            //     }
            //     if(!CollideCheck(timesSub, minco_br.Sub.getCoeffs(), hPolyIdxSub, hPolytopesSub, smoothEps, integralRes, magnitudeBd,
            //     penaltyWt, swarmTrajs_->seg_intersec_ids_sub_, swarmTrajs_, stemT)){
            //         cout<<"sub collide"<<endl;
            //         minCostFunctional = INFINITY;
            //     }

            //     std::cout<<"exceed max iter!!!!!!!!!!!!"<<std::endl;
            // }
            else
            {
                std::cout<<"ret:"<<ret<<std::endl;

                // cout<<"x:"<<x.transpose()<<endl;
                // trajMs.clear();
                trajSub.clear();
                trajStem.clear();
                trajMain.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }

            return minCostFunctional;
        }
    };

}

#endif
