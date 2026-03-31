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

#ifndef GCOPTER_NORM_HPP
#define GCOPTER_NORM_HPP

#include "gcopter/minco.hpp"
#include "gcopter/geo_utils.hpp"
#include "gcopter/lbfgs.hpp"
#include "gcopter/fastcheck_traj.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

using namespace std;
namespace gcopter_norm
{
    class GCOPTER_NORM
    {
    public:
        typedef Eigen::Matrix3Xd PolyhedronV;
        typedef Eigen::MatrixX4d PolyhedronH;
        typedef std::vector<PolyhedronV> PolyhedraV;
        typedef std::vector<PolyhedronH> PolyhedraH;

    private:
        // minco::MINCO_S3NU minco;
        minco::MINCO_S3NU_PY minco;
        // flatness::FlatnessMap flatmap;

        double rho;
        Eigen::Matrix3Xd vVel;
        Eigen::Vector2d Dy;


        Eigen::Matrix4Xd headPVA; // init
        Eigen::Matrix4Xd tailPVA; // end

        Eigen::Matrix3Xd vEndVel;
        PolyhedraV vPolytopesNorm;
        PolyhedraH hPolytopesNorm;
        // std::vector<Trajectory4<5>> *swarm_trajs_;
        SWARM_TRAJs *swarmTrajs_;
        
        Eigen::Matrix3Xd shortPathNorm;
        Eigen::VectorXd yawPathNorm;
        std::vector<std::pair<double, double>> yawRangeNorm;

        Eigen::VectorXi pieceIdxNorm; 
        Eigen::VectorXi vPolyIdxNorm;
        Eigen::VectorXi hPolyIdxNorm;

        int pieceNNorm;

        int spatialDimNorm;
        int temporalDimNorm;
        int yawDimNorm;



        double smoothEps;
        int integralRes;
        Eigen::VectorXd magnitudeBd;
        Eigen::VectorXd penaltyWt;
        // Eigen::VectorXd physicalPm;
        double allocSpeed, allocYawSpeed;

        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix4Xd pointsNorm; //stem interpoints
        Eigen::VectorXd timesNorm;      // stem times
        Eigen::Matrix4Xd gradByPointsNorm; 
        Eigen::VectorXd gradByTimesNorm;

        Eigen::MatrixX4d partialGradByCoeffsNorm; 
        Eigen::VectorXd partialGradByTimesNorm; 

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

        static inline void forwardV(const Eigen::Matrix3Xd &vVelocity,
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
                                      &GCOPTER_NORM::costTinyNLS,
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
        static inline void backwardGradV(const Eigen::VectorXd &gradv,
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
        static inline void normRetrictionLayerV(const Eigen::VectorXd &vi,
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
                                                const double &smoothFactor,
                                                const int &integralResolution,
                                                const Eigen::VectorXd &magnitudeBounds,
                                                const Eigen::VectorXd &penaltyWeights,
                                                SWARM_TRAJs *swarmTrajs,
                                                // const vector<Trajectory4<5>> *&swarmTrajs,
                                                // const vector<pair<double, double>> &swarmTrajsStartEndTimes,
                                                // const vector<Eigen::Vector3d> &swarmPos,
                                                const double &t0,
                                                double &cost,
                                                Eigen::VectorXd &gradT,
                                                Eigen::MatrixX4d &gradC){
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
            double violaPos, violaVel, violaAcc, violaJer, violaDyaw, violaDdyaw, violaSwarm;
            double violaPosPena, violaVelPena, violaAccPena, violaJerPena, violaDyawPena, violaDdyawPena, violaSwarmPena;
            double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaJerPenaD, violaDyawPenaD, violaDdyawPenaD, violaSwarmPenaD;

            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;
            
            // D = swarmTrajs->trajs_.size();
            S = swarmTrajs->SwarmPos_.size();
            tse = 0.0;


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

                    gradPos.setZero(), gradVel.setZero(), gradAcc.setZero(), gradJer.setZero();
                    pena = 0.0;

                    tc = tse + j * step + t0;

                    // swarm traj cost
                    for(auto &id : swarmTrajs->seg_intersec_ids_norm_[i]){
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

        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        // static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
        //                                            const Eigen::MatrixX3d &coeffs,
        //                                            const Eigen::VectorXi &hIdx,
        //                                            const PolyhedraH &hPolys,
        //                                            const double &smoothFactor,
        //                                            const int &integralResolution,
        //                                            const Eigen::VectorXd &magnitudeBounds,
        //                                            const Eigen::VectorXd &penaltyWeights,
        //                                            flatness::FlatnessMap &flatMap,
        //                                            double &cost,
        //                                            Eigen::VectorXd &gradT,
        //                                            Eigen::MatrixX3d &gradC)
        // {
        //     const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
        //     const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
        //     const double thetaMax = magnitudeBounds(2);
        //     const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));
        //     const double thrustRadi = 0.5 * fabs(magnitudeBounds(4) - magnitudeBounds(3));
        //     const double thrustSqrRadi = thrustRadi * thrustRadi;

        //     const double weightPos = penaltyWeights(0);
        //     const double weightVel = penaltyWeights(1);
        //     const double weightOmg = penaltyWeights(2);
        //     const double weightTheta = penaltyWeights(3);
        //     const double weightThrust = penaltyWeights(4);

        //     Eigen::Vector3d pos, vel, acc, jer, sna;
        //     Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
        //     double totalGradPsi, totalGradPsiD;
        //     double thr, cos_theta;
        //     Eigen::Vector4d quat;
        //     Eigen::Vector3d omg;
        //     double gradThr;
        //     Eigen::Vector4d gradQuat;
        //     Eigen::Vector3d gradPos, gradVel, gradOmg;

        //     double step, alpha;
        //     double s1, s2, s3, s4, s5;
        //     Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
        //     Eigen::Vector3d outerNormal;
        //     int K, L;
        //     double violaPos, violaVel, violaOmg, violaTheta, violaThrust;
        //     double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaThetaPenaD, violaThrustPenaD;
        //     double violaPosPena, violaVelPena, violaOmgPena, violaThetaPena, violaThrustPena;
        //     double node, pena;

        //     const int pieceNum = T.size();
        //     const double integralFrac = 1.0 / integralResolution;
        //     for (int i = 0; i < pieceNum; i++)
        //     {
        //         const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
        //         step = T(i) * integralFrac;
        //         for (int j = 0; j <= integralResolution; j++)
        //         {
        //             s1 = j * step;
        //             s2 = s1 * s1;
        //             s3 = s2 * s1;
        //             s4 = s2 * s2;
        //             s5 = s4 * s1;
        //             beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
        //             beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
        //             beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
        //             beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
        //             beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
        //             pos = c.transpose() * beta0;
        //             vel = c.transpose() * beta1;
        //             acc = c.transpose() * beta2;
        //             jer = c.transpose() * beta3;
        //             sna = c.transpose() * beta4;

        //             flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);

        //             violaVel = vel.squaredNorm() - velSqrMax;
        //             violaOmg = omg.squaredNorm() - omgSqrMax;
        //             cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
        //             violaTheta = acos(cos_theta) - thetaMax;
        //             violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

        //             gradThr = 0.0;
        //             gradQuat.setZero();
        //             gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
        //             pena = 0.0;

        //             L = hIdx(i);
        //             K = hPolys[L].rows();
        //             for (int k = 0; k < K; k++)
        //             {
        //                 outerNormal = hPolys[L].block<1, 3>(k, 0);
        //                 violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
        //                 if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
        //                 {
        //                     gradPos += weightPos * violaPosPenaD * outerNormal;
        //                     pena += weightPos * violaPosPena;
        //                 }
        //             }

        //             if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
        //             {
        //                 gradVel += weightVel * violaVelPenaD * 2.0 * vel;
        //                 pena += weightVel * violaVelPena;
        //             }

        //             if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
        //             {
        //                 gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
        //                 pena += weightOmg * violaOmgPena;
        //             }

        //             if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
        //             {
        //                 gradQuat += weightTheta * violaThetaPenaD /
        //                             sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
        //                             Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
        //                 pena += weightTheta * violaThetaPena;
        //             }

        //             if (smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD))
        //             {
        //                 gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);
        //                 pena += weightThrust * violaThrustPena;
        //             }

        //             flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg,
        //                              totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
        //                              totalGradPsi, totalGradPsiD);

        //             node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
        //             alpha = j * integralFrac;
        //             gradC.block<6, 3>(i * 6, 0) += (beta0 * totalGradPos.transpose() +
        //                                             beta1 * totalGradVel.transpose() +
        //                                             beta2 * totalGradAcc.transpose() +
        //                                             beta3 * totalGradJer.transpose()) *
        //                                            node * step;
        //             gradT(i) += (totalGradPos.dot(vel) +
        //                          totalGradVel.dot(acc) +
        //                          totalGradAcc.dot(jer) +
        //                          totalGradJer.dot(sna)) *
        //                             alpha * node * step +
        //                         node * integralFrac * pena;
        //             cost += node * step * pena;
        //         }
        //     }

        //     return;
        // }

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
            GCOPTER_NORM &obj = *(GCOPTER_NORM *)ptr;

            const int dTauSt = obj.temporalDimNorm;
            const int dXSt = obj.spatialDimNorm;
            const int dYSt = obj.yawDimNorm;

            const double weightT = obj.rho;

            int s = 0;
            Eigen::Map<const Eigen::VectorXd> tauNorm(x.data() + s, dTauSt);
            s += dTauSt;

            Eigen::Map<const Eigen::VectorXd> xiNorm(x.data() + s, dXSt);
            s += dXSt;

            Eigen::Map<const Eigen::VectorXd> yiNorm(x.data() + s, dYSt);

            s = 0;

            Eigen::Map<Eigen::VectorXd> gradTauNorm(g.data() + s, dTauSt);
            s += dTauSt;



            Eigen::Map<Eigen::VectorXd> gradXiNorm(g.data() + s, dXSt);
            s += dXSt;

            Eigen::Map<Eigen::VectorXd> gradYiNorm(g.data() + s, dYSt);

            forwardT(tauNorm, obj.timesNorm);
            forwardP(xiNorm, obj.vPolyIdxNorm, obj.vPolytopesNorm, obj.yawRangeNorm, yiNorm, obj.pointsNorm);

            double cost;
            double cep = 0, cey = 0;
            Eigen::MatrixX3d gpNorm;
            Eigen::VectorXd gyNorm, gtNorm;
            obj.minco.setParameters(obj.pointsNorm, obj.timesNorm);
            obj.minco.getEnergyPos(cep);
            obj.minco.getEnergyYaw(cey);
            cost = cep + cey * 0.25;
            obj.minco.getEnergyPartialGradByCoeffsPos(gpNorm);
            obj.partialGradByCoeffsNorm.leftCols(3) = gpNorm;
            obj.minco.getEnergyPartialGradByCoeffsYaw(gyNorm);
            obj.partialGradByCoeffsNorm.col(3) = gyNorm * 0.25;
            obj.minco.getEnergyPosPartialGradByTimes(gtNorm);
            obj.partialGradByTimesNorm = gtNorm;
            obj.minco.getEnergyYawPartialGradByTimes(gtNorm);
            obj.partialGradByTimesNorm += gtNorm * 0.25;

            attachPenaltyFunctional(obj.timesNorm, obj.minco.getCoeffs(),
                                    obj.hPolyIdxNorm, obj.hPolytopesNorm,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.swarmTrajs_, 0.0,
                                    cost, obj.partialGradByTimesNorm, 
                                    obj.partialGradByCoeffsNorm);

            obj.minco.propogateGrad(obj.partialGradByCoeffsNorm, obj.partialGradByTimesNorm, obj.gradByPointsNorm, obj.gradByTimesNorm);

            cost += weightT * obj.timesNorm.sum();
            obj.gradByTimesNorm.array() += weightT;

            backwardGradT(tauNorm, obj.gradByTimesNorm, gradTauNorm);

            backwardGradP(xiNorm, obj.vPolyIdxNorm, obj.vPolytopesNorm, yiNorm, obj.yawRangeNorm, obj.gradByPointsNorm, gradXiNorm, gradYiNorm);

            normRetrictionLayer(xiNorm, obj.vPolyIdxNorm, obj.vPolytopesNorm, yiNorm, cost, gradXiNorm, gradYiNorm);
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
                                  &GCOPTER_NORM::costDistance,
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
                timeAlloc.segment(j, l).setConstant(std::max(c.head(3).norm() / speed, abs(c(3)) / yawSpeed));
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
            const double velSqrMax = (magnitudeBounds(0) + 2.0) * (magnitudeBounds(0) + 2.0);
            const double accSqrMax = (magnitudeBounds(1) + 2.5) * (magnitudeBounds(1) + 2.5);
            // const double dyawMax = magnitudeBounds(3) * magnitudeBounds(3);
            // const double ddyawMax = magnitudeBounds(4) * magnitudeBounds(4);
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
                        if(i == 0) continue;
                        outerNormal = hPolys[L].block<1, 3>(k, 0);
                        violaPos = outerNormal.dot(pos.head(3)) + hPolys[L](k, 3) - 0.05;
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            std::cout<<"viola pos:"<<pos.transpose()<<std::endl;
                            return false;
                        }
                    }

                    if(vel.head(3).squaredNorm() - velSqrMax > 0.0){
                        std::cout<<"viola vel:"<<vel.norm()<<std::endl;
                        return false;
                    }
                    if(acc.head(3).squaredNorm() - accSqrMax > 0.0){
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

    public:

        /**
         * @brief set path, corridors and optimization params
         * 
         */
        inline bool setup(const double &timeWeight,
                          const Eigen::Matrix4Xd &initialPVA,
                        //   const Eigen::Matrix3d &terminalPVA,
                          const Eigen::Vector4d &endP,
                          const PolyhedraH &corridorNorm,
                          const PolyhedraV &CorridorNormV,
                        //   const PolyhedraH &corridorMain,
                        //   const PolyhedraV &CorridorMainV,
                        //   const PolyhedraH &corridorSub,
                        //   const PolyhedraV &CorridorSubV,
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
            headPVA.resize(4, 3);
            headPVA = initialPVA;
            tailPVA.resize(4, 3);
            tailPVA.setZero();
            tailPVA.col(0) = endP;

            hPolytopesNorm = corridorNorm;
            vPolytopesNorm = CorridorNormV;
            for (size_t i = 0; i < hPolytopesNorm.size(); i++)
            {
                const Eigen::ArrayXd norms =
                hPolytopesNorm[i].leftCols<3>().rowwise().norm();
                hPolytopesNorm[i].array().colwise() /= norms;
            }

            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            // physicalPm = physicalParams;
            allocSpeed = magnitudeBd(0) * 0.75;
            allocYawSpeed = magnitudeBd(3) * 0.75;
            // sub init
            int normPolyN = hPolytopesNorm.size();
            getShortestPath(headPVA.col(0).head(3), tailPVA.col(0).head(3),
                            vPolytopesNorm, smoothEps, shortPathNorm);
            const Eigen::Matrix3Xd deltasNorm = shortPathNorm.rightCols(normPolyN) - shortPathNorm.leftCols(normPolyN);
            pieceIdxNorm = (deltasNorm.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            const Eigen::VectorXd lengthsNorm = deltasNorm.colwise().norm().transpose();
            double total_l_norm = deltasNorm.colwise().norm().sum();

            pieceIdxNorm.array() += 1;
            pieceNNorm = pieceIdxNorm.sum();

            temporalDimNorm = pieceNNorm;
            spatialDimNorm = 0;
            yawDimNorm = 0;
            vPolyIdxNorm.resize(pieceNNorm - 1);
            hPolyIdxNorm.resize(pieceNNorm);

            yawPathNorm.resize(pieceNNorm + 1);
            yawPathNorm(0) = headPVA(3, 0);
            yawRangeNorm.resize(pieceNNorm - 1);
            double dyaw = tailPVA(3, 0) - yawPathNorm(0);
            double ll = 0;

            for (int i = 0, j = 0, k; i < normPolyN; i++)
            {
                k = pieceIdxNorm(i);
                for (int l = 0; l < k; l++, j++)
                {
                    yawPathNorm(j + 1) = dyaw * ((l + 1.0) / k * lengthsNorm(i) + ll) / total_l_norm + yawPathNorm(0);

                    if (l < k - 1)
                    {
                        yawRangeNorm[j].first = yawPathNorm(j + 1) - 1.5;
                        yawRangeNorm[j].second = 3.0;
                        yawDimNorm += 2;
                        vPolyIdxNorm(j) = 2 * i;
                        spatialDimNorm += vPolytopesNorm[2 * i].cols();
                    }
                    else if (i < normPolyN - 1)
                    {
                        yawRangeNorm[j].first = yawPathNorm(j + 1) - 1.5;
                        yawRangeNorm[j].second = 3.0;
                        yawDimNorm += 2;
                        vPolyIdxNorm(j) = 2 * i + 1;
                        spatialDimNorm += vPolytopesNorm[2 * i + 1].cols();
                    }
                    hPolyIdxNorm(j) = i;
                }
                ll += lengthsNorm(i);
            }

            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco.setPieces(pieceNNorm);
            minco.setInitConditions(headPVA);
            minco.setTermConditions(tailPVA);
             
            // Allocate temp variables
            pointsNorm.resize(4, pieceNNorm - 1);

            timesNorm.resize(pieceNNorm);

            gradByPointsNorm.resize(4, pieceNNorm - 1);
            gradByTimesNorm.resize(pieceNNorm);
            partialGradByCoeffsNorm.resize(6 * pieceNNorm, 4);
            partialGradByTimesNorm.resize(pieceNNorm);

            return true;
        }

        /**
         * @brief call after setup()
         * 
         * @param trajNorm       the optimized normal traj   
         * @param relCostTol 
         * @return double 
         */
        inline double optimize(Trajectory4<5> &trajNorm,
                               const double &relCostTol)
        {

            Eigen::VectorXd xBest;
            Eigen::VectorXd x(temporalDimNorm + spatialDimNorm + yawDimNorm);
            int s = 0;
            Eigen::Map<Eigen::VectorXd> tauNorm(x.data() + s, temporalDimNorm);
            s += temporalDimNorm;

            Eigen::Map<Eigen::VectorXd> xiNorm(x.data() + s, spatialDimNorm);
            s += spatialDimNorm;

            Eigen::Map<Eigen::VectorXd> yiNorm(x.data() + s, yawDimNorm);
            s += yawDimNorm;

            //points are downsampled points of shortPath 
            setInitial(shortPathNorm, allocSpeed, yawPathNorm, allocYawSpeed, pieceIdxNorm, pointsNorm, timesNorm);
            timesNorm.array() += 0.025 / timesNorm.size();
            // setInitial(shortPathSub, allocSpeed, pieceIdxSub, pointsSub, timesSub);
            // setInitial(shortPathStem, allocSpeed, pieceIdxStem, pointsStem, timesStem);
            // setInitial(shortPathMain, allocSpeed, pieceIdxMain, pointsMain, timesMain);
            backwardT(timesNorm, tauNorm);
            backwardP(pointsNorm, vPolyIdxNorm, vPolytopesNorm, xiNorm);
            // evi.setOnes();
            yiNorm.setOnes();
            yiNorm = yiNorm / sqrt(yiNorm.size() + 1e-3);

            double minCostFunctional;
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 35;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;
            // lbfgs_params.delta = 1.0e-2;
            int ret;
            double t_best = 999999.0;


            for(int i = 0; i < 4; i++){
                ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GCOPTER_NORM::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);
                forwardT(tauNorm, timesNorm);
                forwardP(xiNorm, vPolyIdxNorm, vPolytopesNorm, yawRangeNorm, yiNorm, pointsNorm);
                minco.setParameters(pointsNorm, timesNorm);
                minco.getTrajectory(trajNorm);
                if(timesNorm.sum() < t_best && CollideCheck(timesNorm, minco.getCoeffs(),
                            hPolyIdxNorm, hPolytopesNorm,
                            smoothEps, integralRes,
                            magnitudeBd, penaltyWt, swarmTrajs_->seg_intersec_ids_norm_, swarmTrajs_, 0.0)){
                    xBest = x;
                    t_best = timesNorm.sum();
                }
                timesNorm *= 0.8;
                backwardT(timesNorm, tauNorm);

            }

            if(t_best < 999998.0){
                x = xBest;
                timesNorm *= 0.75;
                backwardT(timesNorm, tauNorm);
            }
            lbfgs_params.max_iterations = 100;
            ret = lbfgs::lbfgs_optimize(x,
                minCostFunctional,
                &GCOPTER_NORM::costFunctional,
                nullptr,
                nullptr,
                this,
                lbfgs_params);
            forwardT(tauNorm, timesNorm);
            forwardP(xiNorm, vPolyIdxNorm, vPolytopesNorm, yawRangeNorm, yiNorm, pointsNorm);
            minco.setParameters(pointsNorm, timesNorm);
            minco.getTrajectory(trajNorm);
            if(timesNorm.sum() < t_best && CollideCheck(timesNorm, minco.getCoeffs(),
                        hPolyIdxNorm, hPolytopesNorm,
                        smoothEps, integralRes,
                        magnitudeBd, penaltyWt, swarmTrajs_->seg_intersec_ids_norm_, swarmTrajs_, 0.0)){
                xBest = x;
                t_best = timesNorm.sum();
            }

            if (t_best < 999998.0)
            {
                s = 0;
                Eigen::Map<Eigen::VectorXd> tauNormBest(xBest.data() + s, temporalDimNorm);
                s += temporalDimNorm;
    
                Eigen::Map<Eigen::VectorXd> xiNormBest(xBest.data() + s, spatialDimNorm);
                s += spatialDimNorm;
    
                Eigen::Map<Eigen::VectorXd> yiNormBest(xBest.data() + s, yawDimNorm);
                s += yawDimNorm;
                forwardT(tauNormBest, timesNorm);
                forwardP(xiNormBest, vPolyIdxNorm, vPolytopesNorm, yawRangeNorm, yiNormBest, pointsNorm);
                minco.setParameters(pointsNorm, timesNorm);
                minco.getTrajectory(trajNorm);
            }
            else
            {
                // trajMs.clear();
                trajNorm.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }


            // if (ret >= 0)
            // {
            //     forwardT(tauNorm, timesNorm);
            //     forwardP(xiNorm, vPolyIdxNorm, vPolytopesNorm, yawRangeNorm, yiNorm, pointsNorm);
            //     minco.setParameters(pointsNorm, timesNorm);
            //     minco.getTrajectory(trajNorm);
            // }
            // else if(ret == lbfgs::LBFGSERR_MAXIMUMITERATION || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
            //     forwardT(tauNorm, timesNorm);
            //     forwardP(xiNorm, vPolyIdxNorm, vPolytopesNorm, yawRangeNorm, yiNorm, pointsNorm);
            //     minco.setParameters(pointsNorm, timesNorm);
            //     minco.getTrajectory(trajNorm);
            //     std::cout<<"exceed max iter!!!!!!!!!!!!"<<std::endl;
            // }
            // else
            // {
            //     // trajMs.clear();
            //     trajNorm.clear();
            //     minCostFunctional = INFINITY;
            //     std::cout << "Optimization Failed: "
            //               << lbfgs::lbfgs_strerror(ret)
            //               << std::endl;
            // }

            return minCostFunctional;
        }
    };

}

#endif
