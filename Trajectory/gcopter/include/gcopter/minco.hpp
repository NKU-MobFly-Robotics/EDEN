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

#ifndef MINCO_HPP
#define MINCO_HPP

#include "gcopter/trajectory.hpp"
#include "gcopter/trajectory4.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <vector>

namespace minco
{

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            return;
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        // Compulsory nullptr initialization here
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solve(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solveAdj(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }
    };

    // MINCO for s=2 and non-uniform time
    class MINCO_S2NU
    {
    public:
        MINCO_S2NU() = default;
        ~MINCO_S2NU() { A.destroy(); }

    private:
        int N;
        Eigen::Matrix<double, 3, 2> headPV;
        Eigen::Matrix<double, 3, 2> tailPV;
        BandedSystem A;
        Eigen::MatrixX3d b;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;

    public:
        inline void setConditions(const Eigen::Matrix<double, 3, 2> &headState,
                                  const Eigen::Matrix<double, 3, 2> &tailState,
                                  const int &pieceNum)
        {
            N = pieceNum;
            headPV = headState;
            tailPV = tailState;
            A.create(4 * N, 4, 4);
            b.resize(4 * N, 3);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            return;
        }

        inline void setParameters(const Eigen::Matrix3Xd &inPs,
                                  const Eigen::VectorXd &ts)
        {
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);

            A.reset();
            b.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            b.row(0) = headPV.col(0).transpose();
            b.row(1) = headPV.col(1).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                A(4 * i + 2, 4 * i + 2) = 2.0;
                A(4 * i + 2, 4 * i + 3) = 6.0 * T1(i);
                A(4 * i + 2, 4 * i + 6) = -2.0;
                A(4 * i + 3, 4 * i) = 1.0;
                A(4 * i + 3, 4 * i + 1) = T1(i);
                A(4 * i + 3, 4 * i + 2) = T2(i);
                A(4 * i + 3, 4 * i + 3) = T3(i);
                A(4 * i + 4, 4 * i) = 1.0;
                A(4 * i + 4, 4 * i + 1) = T1(i);
                A(4 * i + 4, 4 * i + 2) = T2(i);
                A(4 * i + 4, 4 * i + 3) = T3(i);
                A(4 * i + 4, 4 * i + 4) = -1.0;
                A(4 * i + 5, 4 * i + 1) = 1.0;
                A(4 * i + 5, 4 * i + 2) = 2.0 * T1(i);
                A(4 * i + 5, 4 * i + 3) = 3.0 * T2(i);
                A(4 * i + 5, 4 * i + 5) = -1.0;

                b.row(4 * i + 3) = inPs.col(i).transpose();
            }

            A(4 * N - 2, 4 * N - 4) = 1.0;
            A(4 * N - 2, 4 * N - 3) = T1(N - 1);
            A(4 * N - 2, 4 * N - 2) = T2(N - 1);
            A(4 * N - 2, 4 * N - 1) = T3(N - 1);
            A(4 * N - 1, 4 * N - 3) = 1.0;
            A(4 * N - 1, 4 * N - 2) = 2 * T1(N - 1);
            A(4 * N - 1, 4 * N - 1) = 3 * T2(N - 1);

            b.row(4 * N - 2) = tailPV.col(0).transpose();
            b.row(4 * N - 1) = tailPV.col(1).transpose();

            A.factorizeLU();
            A.solve(b);

            return;
        }

        inline void getTrajectory(Trajectory<3> &traj) const
        {
            traj.clear();
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i),
                                  b.block<4, 3>(4 * i, 0)
                                      .transpose()
                                      .rowwise()
                                      .reverse());
            }
            return;
        }

        inline void getEnergy(double &energy) const
        {
            energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                energy += 4.0 * b.row(4 * i + 2).squaredNorm() * T1(i) +
                          12.0 * b.row(4 * i + 2).dot(b.row(4 * i + 3)) * T2(i) +
                          12.0 * b.row(4 * i + 3).squaredNorm() * T3(i);
            }
            return;
        }

        inline const Eigen::MatrixX3d &getCoeffs(void) const
        {
            return b;
        }

        inline void getEnergyPartialGradByCoeffs(Eigen::MatrixX3d &gdC) const
        {
            gdC.resize(4 * N, 3);
            for (int i = 0; i < N; i++)
            {
                gdC.row(4 * i + 3) = 12.0 * b.row(4 * i + 2) * T2(i) +
                                     24.0 * b.row(4 * i + 3) * T3(i);
                gdC.row(4 * i + 2) = 8.0 * b.row(4 * i + 2) * T1(i) +
                                     12.0 * b.row(4 * i + 3) * T2(i);
                gdC.block<2, 3>(4 * i, 0).setZero();
            }
            return;
        }

        inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
        {
            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                gdT(i) = 4.0 * b.row(4 * i + 2).squaredNorm() +
                         24.0 * b.row(4 * i + 2).dot(b.row(4 * i + 3)) * T1(i) +
                         36.0 * b.row(4 * i + 3).squaredNorm() * T2(i);
            }
            return;
        }

        inline void propogateGrad(const Eigen::MatrixX3d &partialGradByCoeffs,
                                  const Eigen::VectorXd &partialGradByTimes,
                                  Eigen::Matrix3Xd &gradByPoints,
                                  Eigen::VectorXd &gradByTimes)

        {
            gradByPoints.resize(3, N - 1);
            gradByTimes.resize(N);
            Eigen::MatrixX3d adjGrad = partialGradByCoeffs;
            A.solveAdj(adjGrad);

            for (int i = 0; i < N - 1; i++)
            {
                gradByPoints.col(i) = adjGrad.row(4 * i + 3).transpose();
            }

            Eigen::Matrix<double, 4, 3> B1;
            Eigen::Matrix<double, 2, 3> B2;
            for (int i = 0; i < N - 1; i++)
            {
                // negative jerk
                B1.row(0) = -6.0 * b.row(i * 4 + 3);

                // negative velocity
                B1.row(1) = -(b.row(i * 4 + 1) +
                              2.0 * T1(i) * b.row(i * 4 + 2) +
                              3.0 * T2(i) * b.row(i * 4 + 3));
                B1.row(2) = B1.row(1);

                // negative acceleration
                B1.row(3) = -(2.0 * b.row(i * 4 + 2) +
                              6.0 * T1(i) * b.row(i * 4 + 3));

                gradByTimes(i) = B1.cwiseProduct(adjGrad.block<4, 3>(4 * i + 2, 0)).sum();
            }

            // negative velocity
            B2.row(0) = -(b.row(4 * N - 3) +
                          2.0 * T1(N - 1) * b.row(4 * N - 2) +
                          3.0 * T2(N - 1) * b.row(4 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(4 * N - 2) +
                          6.0 * T1(N - 1) * b.row(4 * N - 1));

            gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<2, 3>(4 * N - 2, 0)).sum();

            gradByTimes += partialGradByTimes;
        }
    };

    // MINCO for s=3 and non-uniform time
    class MINCO_S3NU
    {
    public:
        MINCO_S3NU() = default;
        ~MINCO_S3NU() { A.destroy(); }

    private:
        int N;
        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;
        BandedSystem A;     //M
        Eigen::MatrixX3d b;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;

    public:
        inline void setEndVel(const Eigen::Vector3d &endv){
            tailPVA.col(1).head(3) = endv;
        }

        inline void setConditions(const Eigen::Matrix3d &headState,
                                  const Eigen::Matrix3d &tailState,
                                  const int &pieceNum)
        {
            N = pieceNum;
            headPVA = headState;
            tailPVA = tailState;
            A.create(6 * N, 6, 6);
            b.resize(6 * N, 3);   //initially b = b; after setParameters, b = c
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);
            return;
        }

        inline void setParameters(const Eigen::Matrix3Xd &inPs,
                                  const Eigen::VectorXd &ts)
        {
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);

            A.reset();
            b.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            b.row(0) = headPVA.col(0).transpose();
            b.row(1) = headPVA.col(1).transpose();
            b.row(2) = headPVA.col(2).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                A(6 * i + 3, 6 * i + 3) = 6.0;
                A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                A(6 * i + 3, 6 * i + 9) = -6.0;
                A(6 * i + 4, 6 * i + 4) = 24.0;
                A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                A(6 * i + 4, 6 * i + 10) = -24.0;
                A(6 * i + 5, 6 * i) = 1.0;
                A(6 * i + 5, 6 * i + 1) = T1(i);
                A(6 * i + 5, 6 * i + 2) = T2(i);
                A(6 * i + 5, 6 * i + 3) = T3(i);
                A(6 * i + 5, 6 * i + 4) = T4(i);
                A(6 * i + 5, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i) = 1.0;
                A(6 * i + 6, 6 * i + 1) = T1(i);
                A(6 * i + 6, 6 * i + 2) = T2(i);
                A(6 * i + 6, 6 * i + 3) = T3(i);
                A(6 * i + 6, 6 * i + 4) = T4(i);
                A(6 * i + 6, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i + 6) = -1.0;
                A(6 * i + 7, 6 * i + 1) = 1.0;
                A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                A(6 * i + 7, 6 * i + 7) = -1.0;
                A(6 * i + 8, 6 * i + 2) = 2.0;
                A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                A(6 * i + 8, 6 * i + 8) = -2.0;

                b.row(6 * i + 5) = inPs.col(i).transpose();
            }

            A(6 * N - 3, 6 * N - 6) = 1.0;
            A(6 * N - 3, 6 * N - 5) = T1(N - 1);
            A(6 * N - 3, 6 * N - 4) = T2(N - 1);
            A(6 * N - 3, 6 * N - 3) = T3(N - 1);
            A(6 * N - 3, 6 * N - 2) = T4(N - 1);
            A(6 * N - 3, 6 * N - 1) = T5(N - 1);
            A(6 * N - 2, 6 * N - 5) = 1.0;
            A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
            A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
            A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
            A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
            A(6 * N - 1, 6 * N - 4) = 2;
            A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
            A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
            A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

            b.row(6 * N - 3) = tailPVA.col(0).transpose();
            b.row(6 * N - 2) = tailPVA.col(1).transpose();
            b.row(6 * N - 1) = tailPVA.col(2).transpose();

            A.factorizeLU();
            A.solve(b);

            return;
        }

        inline void getTrajectory(Trajectory<5> &traj) const
        {
            traj.clear();
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i),
                                  b.block<6, 3>(6 * i, 0)
                                      .transpose()
                                      .rowwise()
                                      .reverse());
            }
            return;
        }

        inline void getEnergy(double &energy) const
        {
            energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                energy += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                          144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                          192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                          240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                          720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                          720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
            }
            return;
        }

        inline const Eigen::MatrixX3d &getCoeffs(void) const
        {
            return b;
        }
        //d(cost)/d(c)
        inline void getEnergyPartialGradByCoeffs(Eigen::MatrixX3d &gdC) const
        {
            gdC.resize(6 * N, 3);
            for (int i = 0; i < N; i++)
            {
                gdC.row(6 * i + 5) = 240.0 * b.row(6 * i + 3) * T3(i) +
                                     720.0 * b.row(6 * i + 4) * T4(i) +
                                     1440.0 * b.row(6 * i + 5) * T5(i);
                gdC.row(6 * i + 4) = 144.0 * b.row(6 * i + 3) * T2(i) +
                                     384.0 * b.row(6 * i + 4) * T3(i) +
                                     720.0 * b.row(6 * i + 5) * T4(i);
                gdC.row(6 * i + 3) = 72.0 * b.row(6 * i + 3) * T1(i) +
                                     144.0 * b.row(6 * i + 4) * T2(i) +
                                     240.0 * b.row(6 * i + 5) * T3(i);
                gdC.block<3, 3>(6 * i, 0).setZero();
            }
            return;
        }
        //d(cost)/d(T)
        inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
        {
            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                gdT(i) = 36.0 * b.row(6 * i + 3).squaredNorm() +
                         288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                         576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                         720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                         2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                         3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
            }
            return;
        }

        inline void propogateGrad(const Eigen::MatrixX3d &partialGradByCoeffs,
                                  const Eigen::VectorXd &partialGradByTimes,
                                  Eigen::Matrix3Xd &gradByPoints,
                                  Eigen::VectorXd &gradByEndVel,
                                  Eigen::VectorXd &gradByTimes)

        {
            gradByPoints.resize(3, N - 1);
            gradByTimes.resize(N);
            Eigen::MatrixX3d adjGrad = partialGradByCoeffs;
            A.solveAdj(adjGrad);

            gradByEndVel = adjGrad.row(6 * N - 2).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
            }

            Eigen::Matrix<double, 6, 3> B1;
            Eigen::Matrix3d B2;
            for (int i = 0; i < N - 1; i++)
            {
                // negative velocity
                B1.row(2) = -(b.row(i * 6 + 1) +
                              2.0 * T1(i) * b.row(i * 6 + 2) +
                              3.0 * T2(i) * b.row(i * 6 + 3) +
                              4.0 * T3(i) * b.row(i * 6 + 4) +
                              5.0 * T4(i) * b.row(i * 6 + 5));
                B1.row(3) = B1.row(2);

                // negative acceleration
                B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                              6.0 * T1(i) * b.row(i * 6 + 3) +
                              12.0 * T2(i) * b.row(i * 6 + 4) +
                              20.0 * T3(i) * b.row(i * 6 + 5));

                // negative jerk
                B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                              24.0 * T1(i) * b.row(i * 6 + 4) +
                              60.0 * T2(i) * b.row(i * 6 + 5));

                // negative snap
                B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                              120.0 * T1(i) * b.row(i * 6 + 5));

                // negative crackle
                B1.row(1) = -120.0 * b.row(i * 6 + 5);

                gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 3>(6 * i + 3, 0)).sum();
            }

            // negative velocity
            B2.row(0) = -(b.row(6 * N - 5) +
                          2.0 * T1(N - 1) * b.row(6 * N - 4) +
                          3.0 * T2(N - 1) * b.row(6 * N - 3) +
                          4.0 * T3(N - 1) * b.row(6 * N - 2) +
                          5.0 * T4(N - 1) * b.row(6 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                          6.0 * T1(N - 1) * b.row(6 * N - 3) +
                          12.0 * T2(N - 1) * b.row(6 * N - 2) +
                          20.0 * T3(N - 1) * b.row(6 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                          24.0 * T1(N - 1) * b.row(6 * N - 2) +
                          60.0 * T2(N - 1) * b.row(6 * N - 1));

            gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 3>(6 * N - 3, 0)).sum();

            gradByTimes += partialGradByTimes;
        }
        


        inline void propogateGrad(const Eigen::MatrixX3d &partialGradByCoeffs,
                                  const Eigen::VectorXd &partialGradByTimes,
                                  Eigen::Matrix3Xd &gradByPoints,
                                  Eigen::VectorXd &gradByTimes)

        {
            gradByPoints.resize(3, N - 1);
            gradByTimes.resize(N);
            Eigen::MatrixX3d adjGrad = partialGradByCoeffs;
            A.solveAdj(adjGrad);

            for (int i = 0; i < N - 1; i++)
            {
                gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
            }

            Eigen::Matrix<double, 6, 3> B1;
            Eigen::Matrix3d B2;
            for (int i = 0; i < N - 1; i++)
            {
                // negative velocity
                B1.row(2) = -(b.row(i * 6 + 1) +
                              2.0 * T1(i) * b.row(i * 6 + 2) +
                              3.0 * T2(i) * b.row(i * 6 + 3) +
                              4.0 * T3(i) * b.row(i * 6 + 4) +
                              5.0 * T4(i) * b.row(i * 6 + 5));
                B1.row(3) = B1.row(2);

                // negative acceleration
                B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                              6.0 * T1(i) * b.row(i * 6 + 3) +
                              12.0 * T2(i) * b.row(i * 6 + 4) +
                              20.0 * T3(i) * b.row(i * 6 + 5));

                // negative jerk
                B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                              24.0 * T1(i) * b.row(i * 6 + 4) +
                              60.0 * T2(i) * b.row(i * 6 + 5));

                // negative snap
                B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                              120.0 * T1(i) * b.row(i * 6 + 5));

                // negative crackle
                B1.row(1) = -120.0 * b.row(i * 6 + 5);

                gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 3>(6 * i + 3, 0)).sum();
            }

            // negative velocity
            B2.row(0) = -(b.row(6 * N - 5) +
                          2.0 * T1(N - 1) * b.row(6 * N - 4) +
                          3.0 * T2(N - 1) * b.row(6 * N - 3) +
                          4.0 * T3(N - 1) * b.row(6 * N - 2) +
                          5.0 * T4(N - 1) * b.row(6 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                          6.0 * T1(N - 1) * b.row(6 * N - 3) +
                          12.0 * T2(N - 1) * b.row(6 * N - 2) +
                          20.0 * T3(N - 1) * b.row(6 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                          24.0 * T1(N - 1) * b.row(6 * N - 2) +
                          60.0 * T2(N - 1) * b.row(6 * N - 1));

            gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 3>(6 * N - 3, 0)).sum();

            gradByTimes += partialGradByTimes;
        }
    };
// MINCO for s=3, with yaw and non-uniform time
class MINCO_S3NU_PY
{
public:
    MINCO_S3NU_PY() = default;
    ~MINCO_S3NU_PY() { A.destroy(); }

private:
    int N;


public:
    Eigen::Matrix4Xd headPVA; // 4 * 3
    Eigen::Matrix4Xd tailPVA;
    BandedSystem A;     //M
    Eigen::MatrixX4d b;
    Eigen::VectorXd T1;
    Eigen::VectorXd T2;
    Eigen::VectorXd T3;
    Eigen::VectorXd T4;
    Eigen::VectorXd T5;

    inline void setEndVel(const Eigen::Vector3d &endv){
        tailPVA.col(1).head(3) = endv;
    }
    inline void setEndDy(const double &endv){
        tailPVA.col(1)(3) = endv;
    }
    inline void setPieces(const int &pieceNum)
    {
        N = pieceNum;

        A.create(6 * N, 6, 6);
        b.resize(6 * N, 4);   //initially b = b; after setParameters, b = c
        T1.resize(N);
        T2.resize(N);
        T3.resize(N);
        T4.resize(N);
        T5.resize(N);
        return;
    }

    inline void setInitConditions(const Eigen::Matrix4Xd &headState){
        headPVA = headState;
    }

    inline void setTermConditions(const Eigen::Matrix4Xd &tailState){
        tailPVA = tailState;
    }

    inline void setParameters(const Eigen::Matrix4Xd &inPs,
                              const Eigen::VectorXd &ts)
    {
        T1 = ts;
        T2 = T1.cwiseProduct(T1);
        T3 = T2.cwiseProduct(T1);
        T4 = T2.cwiseProduct(T2);
        T5 = T4.cwiseProduct(T1);

        A.reset();
        b.setZero();

        A(0, 0) = 1.0;
        A(1, 1) = 1.0;
        A(2, 2) = 2.0;
        b.row(0) = headPVA.col(0).transpose();
        b.row(1) = headPVA.col(1).transpose();
        b.row(2) = headPVA.col(2).transpose();

        for (int i = 0; i < N - 1; i++)
        {
            A(6 * i + 3, 6 * i + 3) = 6.0;
            A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
            A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
            A(6 * i + 3, 6 * i + 9) = -6.0;
            A(6 * i + 4, 6 * i + 4) = 24.0;
            A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
            A(6 * i + 4, 6 * i + 10) = -24.0;
            A(6 * i + 5, 6 * i) = 1.0;
            A(6 * i + 5, 6 * i + 1) = T1(i);
            A(6 * i + 5, 6 * i + 2) = T2(i);
            A(6 * i + 5, 6 * i + 3) = T3(i);
            A(6 * i + 5, 6 * i + 4) = T4(i);
            A(6 * i + 5, 6 * i + 5) = T5(i);
            A(6 * i + 6, 6 * i) = 1.0;
            A(6 * i + 6, 6 * i + 1) = T1(i);
            A(6 * i + 6, 6 * i + 2) = T2(i);
            A(6 * i + 6, 6 * i + 3) = T3(i);
            A(6 * i + 6, 6 * i + 4) = T4(i);
            A(6 * i + 6, 6 * i + 5) = T5(i);
            A(6 * i + 6, 6 * i + 6) = -1.0;
            A(6 * i + 7, 6 * i + 1) = 1.0;
            A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
            A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
            A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
            A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
            A(6 * i + 7, 6 * i + 7) = -1.0;
            A(6 * i + 8, 6 * i + 2) = 2.0;
            A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
            A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
            A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
            A(6 * i + 8, 6 * i + 8) = -2.0;

            b.row(6 * i + 5) = inPs.col(i).transpose();
        }

        A(6 * N - 3, 6 * N - 6) = 1.0;
        A(6 * N - 3, 6 * N - 5) = T1(N - 1);
        A(6 * N - 3, 6 * N - 4) = T2(N - 1);
        A(6 * N - 3, 6 * N - 3) = T3(N - 1);
        A(6 * N - 3, 6 * N - 2) = T4(N - 1);
        A(6 * N - 3, 6 * N - 1) = T5(N - 1);
        A(6 * N - 2, 6 * N - 5) = 1.0;
        A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
        A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
        A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
        A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
        A(6 * N - 1, 6 * N - 4) = 2;
        A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
        A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
        A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

        b.row(6 * N - 3) = tailPVA.col(0).transpose();
        b.row(6 * N - 2) = tailPVA.col(1).transpose();
        b.row(6 * N - 1) = tailPVA.col(2).transpose();

        A.factorizeLU();
        A.solve(b);

        return;
    }

    inline void getTrajectory(Trajectory4<5> &traj) const
    {
        traj.clear();
        traj.reserve(N);
        for (int i = 0; i < N; i++)
        {
            traj.emplace_back(T1(i),
                              b.block<6, 4>(6 * i, 0)
                                  .transpose()
                                  .rowwise()
                                  .reverse());
        }
        return;
    }

    inline void getEnergyYaw(double &energy) const
    {
        Eigen::VectorXd B;
        energy = 0.0;
        B = b.col(3);
        for (int i = 0; i < N; i++)
        {
            energy += 36.0 * B(6 * i + 3) * B(6 * i + 3) * T1(i) +
                      144.0 * B(6 * i + 4) * (B(6 * i + 3)) * T2(i) +
                      192.0 * B(6 * i + 4) * B(6 * i + 4) * T3(i) +
                      240.0 * B(6 * i + 5)*(B(6 * i + 3)) * T3(i) +
                      720.0 * B(6 * i + 5)*(B(6 * i + 4)) * T4(i) +
                      720.0 * B(6 * i + 5) * B(6 * i + 5) * T5(i);
        }
        return;
    }

    inline void getEnergyPos(double &energy) const
    {
        energy = 0.0;
        Eigen::MatrixX3d B;
        B = b.leftCols(3);
        for (int i = 0; i < N; i++)
        {
            energy += 36.0 * B.row(6 * i + 3).squaredNorm() * T1(i) +
                      144.0 * B.row(6 * i + 4).dot(B.row(6 * i + 3)) * T2(i) +
                      192.0 * B.row(6 * i + 4).squaredNorm() * T3(i) +
                      240.0 * B.row(6 * i + 5).dot(B.row(6 * i + 3)) * T3(i) +
                      720.0 * B.row(6 * i + 5).dot(B.row(6 * i + 4)) * T4(i) +
                      720.0 * B.row(6 * i + 5).squaredNorm() * T5(i);
        }
        return;
    }

    inline void getEnergy(double &energy) const
    {
        energy = 0.0;
        for (int i = 0; i < N; i++)
        {
            energy += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                      144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                      192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                      240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                      720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                      720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
        }
        return;
    }

    inline const Eigen::MatrixX4d &getCoeffs(void) const
    {
        return b;
    }
    //d(cost)/d(c)
    inline void getEnergyPartialGradByCoeffs(Eigen::MatrixX4d &gdC) const
    {
        gdC.resize(6 * N, 4);
        for (int i = 0; i < N; i++)
        {
            gdC.row(6 * i + 5) = 240.0 * b.row(6 * i + 3) * T3(i) +
                                 720.0 * b.row(6 * i + 4) * T4(i) +
                                 1440.0 * b.row(6 * i + 5) * T5(i);
            gdC.row(6 * i + 4) = 144.0 * b.row(6 * i + 3) * T2(i) +
                                 384.0 * b.row(6 * i + 4) * T3(i) +
                                 720.0 * b.row(6 * i + 5) * T4(i);
            gdC.row(6 * i + 3) = 72.0 * b.row(6 * i + 3) * T1(i) +
                                 144.0 * b.row(6 * i + 4) * T2(i) +
                                 240.0 * b.row(6 * i + 5) * T3(i);
            gdC.block<3, 4>(6 * i, 0).setZero();
        }
        return;
    }

    inline void getEnergyPartialGradByCoeffsPos(Eigen::MatrixX3d &gdC) const
    {
        gdC.resize(6 * N, 3);
        Eigen::MatrixX3d B;
        B = b.leftCols(3);
        for (int i = 0; i < N; i++)
        {
            gdC.row(6 * i + 5) = 240.0 * B.row(6 * i + 3) * T3(i) +
                                 720.0 * B.row(6 * i + 4) * T4(i) +
                                 1440.0 * B.row(6 * i + 5) * T5(i);
            gdC.row(6 * i + 4) = 144.0 * B.row(6 * i + 3) * T2(i) +
                                 384.0 * B.row(6 * i + 4) * T3(i) +
                                 720.0 * B.row(6 * i + 5) * T4(i);
            gdC.row(6 * i + 3) = 72.0 * B.row(6 * i + 3) * T1(i) +
                                 144.0 * B.row(6 * i + 4) * T2(i) +
                                 240.0 * B.row(6 * i + 5) * T3(i);
            gdC.block<3, 3>(6 * i, 0).setZero();
        }
        return;
    }

    inline void getEnergyPartialGradByCoeffsYaw(Eigen::VectorXd &gdC) const
    {
        gdC.resize(6 * N, 1);
        Eigen::VectorXd B;
        B = b.col(3);
        for (int i = 0; i < N; i++)
        {
            gdC(6 * i + 5) = 240.0 * B(6 * i + 3) * T3(i) +
                                 720.0 * B(6 * i + 4) * T4(i) +
                                 1440.0 * B(6 * i + 5) * T5(i);
            gdC(6 * i + 4) = 144.0 * B(6 * i + 3) * T2(i) +
                                 384.0 * B(6 * i + 4) * T3(i) +
                                 720.0 * B(6 * i + 5) * T4(i);
            gdC(6 * i + 3) = 72.0 * B(6 * i + 3) * T1(i) +
                                 144.0 * B(6 * i + 4) * T2(i) +
                                 240.0 * B(6 * i + 5) * T3(i);
            gdC.block<3, 1>(6 * i, 0).setZero();
        }
        return;
    }
    //d(cost)/d(T)
    inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
    {
        gdT.resize(N);
        for (int i = 0; i < N; i++)
        {
            gdT(i) = 36.0 * b.row(6 * i + 3).squaredNorm() +
                     288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                     576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                     720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                     2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                     3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
        }
        return;
    }

    inline void getEnergyPosPartialGradByTimes(Eigen::VectorXd &gdT) const
    {
        gdT.resize(N);
        gdT.setZero();
        Eigen::MatrixX3d B;
        B = b.leftCols(3);
        for (int i = 0; i < N; i++)
        {
            gdT(i) += 36.0 * B.row(6 * i + 3).squaredNorm() +
                     288.0 * B.row(6 * i + 4).dot(B.row(6 * i + 3)) * T1(i) +
                     576.0 * B.row(6 * i + 4).squaredNorm() * T2(i) +
                     720.0 * B.row(6 * i + 5).dot(B.row(6 * i + 3)) * T2(i) +
                     2880.0 * B.row(6 * i + 5).dot(B.row(6 * i + 4)) * T3(i) +
                     3600.0 * B.row(6 * i + 5).squaredNorm() * T4(i);
        }
        return;
    }

    inline void getEnergyYawPartialGradByTimes(Eigen::VectorXd &gdT) const
    {
        gdT.resize(N);
        gdT.setZero();
        Eigen::VectorXd B;
        B = b.col(3);
        for (int i = 0; i < N; i++)
        {
            gdT(i) += 36.0 * B(6 * i + 3) * B(6 * i + 3) +
                     288.0 * B(6 * i + 4) * B(6 * i + 3) * T1(i) +
                     576.0 * B(6 * i + 4) * B(6 * i + 4) * T2(i) +
                     720.0 * B(6 * i + 5) * B(6 * i + 3) * T2(i) +
                     2880.0 * B(6 * i + 5) * B(6 * i + 4) * T3(i) +
                     3600.0 * B(6 * i + 5) * B(6 * i + 5) * T4(i);
        }
        return;
    }

    inline void propogateGrad(const Eigen::MatrixX4d &partialGradByCoeffs,
                              const Eigen::VectorXd &partialGradByTimes,
                              Eigen::Matrix4Xd &gradByPoints,
                              Eigen::VectorXd &gradByEndVel,
                              Eigen::VectorXd &gradByTimes)

    {
        gradByPoints.resize(4, N - 1);
        gradByTimes.resize(N);
        Eigen::MatrixX4d adjGrad = partialGradByCoeffs;
        A.solveAdj(adjGrad);

        gradByEndVel = adjGrad.row(6 * N - 2).transpose();
        for (int i = 0; i < N - 1; i++)
        {
            gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
        }

        Eigen::Matrix<double, 6, 4> B1;
        Eigen::Matrix<double, 3, 4> B2;
        for (int i = 0; i < N - 1; i++)
        {
            // negative velocity
            B1.row(2) = -(b.row(i * 6 + 1) +
                          2.0 * T1(i) * b.row(i * 6 + 2) +
                          3.0 * T2(i) * b.row(i * 6 + 3) +
                          4.0 * T3(i) * b.row(i * 6 + 4) +
                          5.0 * T4(i) * b.row(i * 6 + 5));
            B1.row(3) = B1.row(2);

            // negative acceleration
            B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                          6.0 * T1(i) * b.row(i * 6 + 3) +
                          12.0 * T2(i) * b.row(i * 6 + 4) +
                          20.0 * T3(i) * b.row(i * 6 + 5));

            // negative jerk
            B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                          24.0 * T1(i) * b.row(i * 6 + 4) +
                          60.0 * T2(i) * b.row(i * 6 + 5));

            // negative snap
            B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                          120.0 * T1(i) * b.row(i * 6 + 5));

            // negative crackle
            B1.row(1) = -120.0 * b.row(i * 6 + 5);

            gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 4>(6 * i + 3, 0)).sum();
        }

        // negative velocity
        B2.row(0) = -(b.row(6 * N - 5) +
                      2.0 * T1(N - 1) * b.row(6 * N - 4) +
                      3.0 * T2(N - 1) * b.row(6 * N - 3) +
                      4.0 * T3(N - 1) * b.row(6 * N - 2) +
                      5.0 * T4(N - 1) * b.row(6 * N - 1));

        // negative acceleration
        B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                      6.0 * T1(N - 1) * b.row(6 * N - 3) +
                      12.0 * T2(N - 1) * b.row(6 * N - 2) +
                      20.0 * T3(N - 1) * b.row(6 * N - 1));

        // negative jerk
        B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                      24.0 * T1(N - 1) * b.row(6 * N - 2) +
                      60.0 * T2(N - 1) * b.row(6 * N - 1));

        gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 4>(6 * N - 3, 0)).sum();

        gradByTimes += partialGradByTimes;
    }

    inline void propogateGradTerm(const Eigen::MatrixX4d &partialGradByCoeffs,
                              const Eigen::VectorXd &partialGradByTimes,
                              Eigen::Matrix4Xd &gradByPoints,
                              Eigen::VectorXd &gradByTimes,
                              Eigen::Matrix<double, 3, 4> &gradByTermStates)
    {
        gradByPoints.resize(4, N - 1);
        gradByTimes.resize(N);
        Eigen::MatrixX4d adjGrad = partialGradByCoeffs;
        A.solveAdj(adjGrad);
        gradByTermStates = adjGrad.bottomRows(3);

        for (int i = 0; i < N - 1; i++)
        {
            gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
        }

        Eigen::Matrix<double, 6, 4> B1;
        Eigen::Matrix<double, 3, 4> B2;
        for (int i = 0; i < N - 1; i++)
        {
            // negative velocity
            B1.row(2) = -(b.row(i * 6 + 1) +
                          2.0 * T1(i) * b.row(i * 6 + 2) +
                          3.0 * T2(i) * b.row(i * 6 + 3) +
                          4.0 * T3(i) * b.row(i * 6 + 4) +
                          5.0 * T4(i) * b.row(i * 6 + 5));
            B1.row(3) = B1.row(2);

            // negative acceleration
            B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                          6.0 * T1(i) * b.row(i * 6 + 3) +
                          12.0 * T2(i) * b.row(i * 6 + 4) +
                          20.0 * T3(i) * b.row(i * 6 + 5));

            // negative jerk
            B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                          24.0 * T1(i) * b.row(i * 6 + 4) +
                          60.0 * T2(i) * b.row(i * 6 + 5));

            // negative snap
            B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                          120.0 * T1(i) * b.row(i * 6 + 5));

            // negative crackle
            B1.row(1) = -120.0 * b.row(i * 6 + 5);

            gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 4>(6 * i + 3, 0)).sum();
        }

        // negative velocity
        B2.row(0) = -(b.row(6 * N - 5) +
                      2.0 * T1(N - 1) * b.row(6 * N - 4) +
                      3.0 * T2(N - 1) * b.row(6 * N - 3) +
                      4.0 * T3(N - 1) * b.row(6 * N - 2) +
                      5.0 * T4(N - 1) * b.row(6 * N - 1));

        // negative acceleration
        B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                      6.0 * T1(N - 1) * b.row(6 * N - 3) +
                      12.0 * T2(N - 1) * b.row(6 * N - 2) +
                      20.0 * T3(N - 1) * b.row(6 * N - 1));

        // negative jerk
        B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                      24.0 * T1(N - 1) * b.row(6 * N - 2) +
                      60.0 * T2(N - 1) * b.row(6 * N - 1));

        gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 4>(6 * N - 3, 0)).sum();

        gradByTimes += partialGradByTimes;
    }


    inline void propogateGradInitTerm(const Eigen::MatrixX4d &partialGradByCoeffs,
        const Eigen::VectorXd &partialGradByTimes,
        Eigen::Matrix4Xd &gradByPoints,
        Eigen::VectorXd &gradByTimes,
        Eigen::Matrix<double, 3, 4> &gradByInitStates,
        Eigen::Matrix<double, 3, 4> &gradByTermStates)

    {
        gradByPoints.resize(4, N - 1);
        gradByTimes.resize(N);
        Eigen::MatrixX4d adjGrad = partialGradByCoeffs;
        A.solveAdj(adjGrad);
        gradByInitStates = adjGrad.block(0, 0, 3, 4);
        gradByTermStates = adjGrad.bottomRows(3);

        for (int i = 0; i < N - 1; i++)
        {
            gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
        }

        Eigen::Matrix<double, 6, 4> B1;
        Eigen::Matrix<double, 3, 4> B2;
        for (int i = 0; i < N - 1; i++)
        {
            // negative velocity
            B1.row(2) = -(b.row(i * 6 + 1) +
                2.0 * T1(i) * b.row(i * 6 + 2) +
                3.0 * T2(i) * b.row(i * 6 + 3) +
                4.0 * T3(i) * b.row(i * 6 + 4) +
                5.0 * T4(i) * b.row(i * 6 + 5));
            B1.row(3) = B1.row(2);

            // negative acceleration
            B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                6.0 * T1(i) * b.row(i * 6 + 3) +
                12.0 * T2(i) * b.row(i * 6 + 4) +
                20.0 * T3(i) * b.row(i * 6 + 5));

            // negative jerk
            B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                24.0 * T1(i) * b.row(i * 6 + 4) +
                60.0 * T2(i) * b.row(i * 6 + 5));

            // negative snap
            B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                120.0 * T1(i) * b.row(i * 6 + 5));

            // negative crackle
            B1.row(1) = -120.0 * b.row(i * 6 + 5);

            gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 4>(6 * i + 3, 0)).sum();
        }

        // negative velocity
        B2.row(0) = -(b.row(6 * N - 5) +
        2.0 * T1(N - 1) * b.row(6 * N - 4) +
        3.0 * T2(N - 1) * b.row(6 * N - 3) +
        4.0 * T3(N - 1) * b.row(6 * N - 2) +
        5.0 * T4(N - 1) * b.row(6 * N - 1));

        // negative acceleration
        B2.row(1) = -(2.0 * b.row(6 * N - 4) +
        6.0 * T1(N - 1) * b.row(6 * N - 3) +
        12.0 * T2(N - 1) * b.row(6 * N - 2) +
        20.0 * T3(N - 1) * b.row(6 * N - 1));

        // negative jerk
        B2.row(2) = -(6.0 * b.row(6 * N - 3) +
        24.0 * T1(N - 1) * b.row(6 * N - 2) +
        60.0 * T2(N - 1) * b.row(6 * N - 1));

        gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 4>(6 * N - 3, 0)).sum();

        gradByTimes += partialGradByTimes;
    }


    inline void propogateGradInit(const Eigen::MatrixX4d &partialGradByCoeffs,
                              const Eigen::VectorXd &partialGradByTimes,
                              Eigen::Matrix4Xd &gradByPoints,
                              Eigen::VectorXd &gradByTimes,
                              Eigen::Matrix<double, 3, 4> &gradByInitStates)

    {
        gradByPoints.resize(4, N - 1);
        gradByTimes.resize(N);
        Eigen::MatrixX4d adjGrad = partialGradByCoeffs;
        A.solveAdj(adjGrad);
        gradByInitStates = adjGrad.block(0, 0, 3, 4);

        for (int i = 0; i < N - 1; i++)
        {
            gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
        }

        Eigen::Matrix<double, 6, 4> B1;
        Eigen::Matrix<double, 3, 4> B2;
        for (int i = 0; i < N - 1; i++)
        {
            // negative velocity
            B1.row(2) = -(b.row(i * 6 + 1) +
                          2.0 * T1(i) * b.row(i * 6 + 2) +
                          3.0 * T2(i) * b.row(i * 6 + 3) +
                          4.0 * T3(i) * b.row(i * 6 + 4) +
                          5.0 * T4(i) * b.row(i * 6 + 5));
            B1.row(3) = B1.row(2);

            // negative acceleration
            B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                          6.0 * T1(i) * b.row(i * 6 + 3) +
                          12.0 * T2(i) * b.row(i * 6 + 4) +
                          20.0 * T3(i) * b.row(i * 6 + 5));

            // negative jerk
            B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                          24.0 * T1(i) * b.row(i * 6 + 4) +
                          60.0 * T2(i) * b.row(i * 6 + 5));

            // negative snap
            B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                          120.0 * T1(i) * b.row(i * 6 + 5));

            // negative crackle
            B1.row(1) = -120.0 * b.row(i * 6 + 5);

            gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 4>(6 * i + 3, 0)).sum();
        }

        // negative velocity
        B2.row(0) = -(b.row(6 * N - 5) +
                      2.0 * T1(N - 1) * b.row(6 * N - 4) +
                      3.0 * T2(N - 1) * b.row(6 * N - 3) +
                      4.0 * T3(N - 1) * b.row(6 * N - 2) +
                      5.0 * T4(N - 1) * b.row(6 * N - 1));

        // negative acceleration
        B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                      6.0 * T1(N - 1) * b.row(6 * N - 3) +
                      12.0 * T2(N - 1) * b.row(6 * N - 2) +
                      20.0 * T3(N - 1) * b.row(6 * N - 1));

        // negative jerk
        B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                      24.0 * T1(N - 1) * b.row(6 * N - 2) +
                      60.0 * T2(N - 1) * b.row(6 * N - 1));

        gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 4>(6 * N - 3, 0)).sum();

        gradByTimes += partialGradByTimes;
    }

    inline void propogateGrad(const Eigen::MatrixX4d &partialGradByCoeffs,
                              const Eigen::VectorXd &partialGradByTimes,
                              Eigen::Matrix4Xd &gradByPoints,
                              Eigen::VectorXd &gradByTimes)

    {
        gradByPoints.resize(4, N - 1);
        gradByTimes.resize(N);
        Eigen::MatrixX4d adjGrad = partialGradByCoeffs;
        A.solveAdj(adjGrad);

        for (int i = 0; i < N - 1; i++)
        {
            gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
        }

        Eigen::Matrix<double, 6, 4> B1;
        Eigen::Matrix<double, 3, 4> B2;
        for (int i = 0; i < N - 1; i++)
        {
            // negative velocity
            B1.row(2) = -(b.row(i * 6 + 1) +
                          2.0 * T1(i) * b.row(i * 6 + 2) +
                          3.0 * T2(i) * b.row(i * 6 + 3) +
                          4.0 * T3(i) * b.row(i * 6 + 4) +
                          5.0 * T4(i) * b.row(i * 6 + 5));
            B1.row(3) = B1.row(2);

            // negative acceleration
            B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                          6.0 * T1(i) * b.row(i * 6 + 3) +
                          12.0 * T2(i) * b.row(i * 6 + 4) +
                          20.0 * T3(i) * b.row(i * 6 + 5));

            // negative jerk
            B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                          24.0 * T1(i) * b.row(i * 6 + 4) +
                          60.0 * T2(i) * b.row(i * 6 + 5));

            // negative snap
            B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                          120.0 * T1(i) * b.row(i * 6 + 5));

            // negative crackle
            B1.row(1) = -120.0 * b.row(i * 6 + 5);

            gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 4>(6 * i + 3, 0)).sum();
        }

        // negative velocity
        B2.row(0) = -(b.row(6 * N - 5) +
                      2.0 * T1(N - 1) * b.row(6 * N - 4) +
                      3.0 * T2(N - 1) * b.row(6 * N - 3) +
                      4.0 * T3(N - 1) * b.row(6 * N - 2) +
                      5.0 * T4(N - 1) * b.row(6 * N - 1));

        // negative acceleration
        B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                      6.0 * T1(N - 1) * b.row(6 * N - 3) +
                      12.0 * T2(N - 1) * b.row(6 * N - 2) +
                      20.0 * T3(N - 1) * b.row(6 * N - 1));

        // negative jerk
        B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                      24.0 * T1(N - 1) * b.row(6 * N - 2) +
                      60.0 * T2(N - 1) * b.row(6 * N - 1));

        gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 4>(6 * N - 3, 0)).sum();

        gradByTimes += partialGradByTimes;
    }
};

    // branch minco
    class MINCO_S4NU_BR{
        public:
        MINCO_S3NU_PY Stem; // stem + main 
        MINCO_S3NU_PY Main; // stem + main 
        MINCO_S3NU_PY Sub; // 
        int stemPieces, mainPieces, subPieces;
        inline void setPieces(const int &stemPieceNum, const int &mainPieceNum, const int &subPieceNum){
            stemPieces = stemPieceNum;
            mainPieces = mainPieceNum;
            subPieces = subPieceNum;
            // int SmPieceNum = stemPieces + mainPieces;
            Stem.setPieces(stemPieces);
            Main.setPieces(mainPieces);
            Sub.setPieces(subPieces);
        }

        // inline void setConditions(const Eigen::Matrix<double, 3, 4> &StemHeadState,
        //     const Eigen::Matrix<double, 3, 4> &MaintailState,
        //     const Eigen::Matrix<double, 3, 4> &SubheadState,
        //     const Eigen::Matrix<double, 3, 4> &SubtailState)
        // {
        //     SteMa.setInitConditions(StemHeadState);
        //     SteMa.setTermConditions(MaintailState);
        //     Sub.setInitConditions(SubheadState);
        //     Sub.setTermConditions(SubtailState);
        //     return;
        // }
        inline void setInterState(){

        }

        inline void setConditions(const Eigen::Matrix4Xd &StemHeadState,
                                    const Eigen::Matrix4Xd &MaintailState,
                                    const Eigen::Matrix4Xd &SubtailState){
            Stem.setInitConditions(StemHeadState);
            Main.setTermConditions(MaintailState);
            Sub.setTermConditions(SubtailState);
        }


        inline void setParameters(
            const Eigen::Matrix4Xd &StemtailState,
            const Eigen::Matrix4Xd &MaintailState,
            const Eigen::MatrixXd &StemPs,
            const Eigen::VectorXd &Stemts,
            const Eigen::MatrixXd &MainPs,
            const Eigen::VectorXd &Maints,
            const Eigen::MatrixXd &SubPs,
            const Eigen::VectorXd &Subts)
        {
            Stem.setTermConditions(StemtailState);
            Main.setInitConditions(StemtailState);
            Main.setTermConditions(MaintailState);
            Sub.setInitConditions(StemtailState);
            Stem.setParameters(StemPs, Stemts);
            // Eigen::Matrix4Xd headPVA_s(4, 3);
            // headPVA_s.col(0) = SteMa.b.row(6 * stemPieces).transpose();
            // headPVA_s.col(1) = SteMa.b.row(6 * stemPieces + 1).transpose();
            // headPVA_s.col(2) = SteMa.b.row(6 * stemPieces + 2).transpose() * 2;

            // Sub.setInitConditions(headPVA_s);
            Sub.setParameters(SubPs, Subts);
            Main.setParameters(MainPs, Maints);
            return;
        }

        inline void setParameters(
                                  const Eigen::Matrix4Xd &StemtailState,
                                  const Eigen::MatrixXd &StemPs,
                                  const Eigen::VectorXd &Stemts,
                                  const Eigen::MatrixXd &MainPs,
                                  const Eigen::VectorXd &Maints,
                                  const Eigen::MatrixXd &SubPs,
                                  const Eigen::VectorXd &Subts)
        {
            Stem.setTermConditions(StemtailState);
            Main.setInitConditions(StemtailState);
            Sub.setInitConditions(StemtailState);
            Stem.setParameters(StemPs, Stemts);
            // Eigen::Matrix4Xd headPVA_s(4, 3);
            // headPVA_s.col(0) = SteMa.b.row(6 * stemPieces).transpose();
            // headPVA_s.col(1) = SteMa.b.row(6 * stemPieces + 1).transpose();
            // headPVA_s.col(2) = SteMa.b.row(6 * stemPieces + 2).transpose() * 2;

            // Sub.setInitConditions(headPVA_s);
            Sub.setParameters(SubPs, Subts);
            Main.setParameters(MainPs, Maints);
            return;
        }

        inline void getEnergyPos(double &energy) const{
            // double em, esu, est;
            // Stem.getEnergyPos(est);
            // Main.getEnergyPos(em);
            // Sub.getEnergyPos(esu);
            // energy = em + esu + est;
            double em, est;
            Stem.getEnergyPos(est);
            Main.getEnergyPos(em);
            // Sub.getEnergyPos(esu);
            energy = em + est;
        }

        inline void getEnergyPartialGradByCoeffsPos(Eigen::MatrixX3d &gdcStem, Eigen::MatrixX3d &gdcMain, Eigen::MatrixX3d &gdcSub) const{
            // Eigen::MatrixX3d gdcMS;
            // SteMa.getEnergyPartialGradByCoeffsPos(gdcMS);
            // gdcStem = gdcMS.topRows(6 * stemPieces);
            // gdcMain = gdcMS.bottomRows(6 * mainPieces);
            Stem.getEnergyPartialGradByCoeffsPos(gdcStem);
            Main.getEnergyPartialGradByCoeffsPos(gdcMain);
            Sub.getEnergyPartialGradByCoeffsPos(gdcSub);
            gdcSub.setZero();
        }

        inline void getEnergyYaw(double &energy) const{
            double em, esu, est;
            Stem.getEnergyYaw(est);
            Main.getEnergyYaw(em);
            Sub.getEnergyYaw(esu);
            energy = em + esu + est;
        }

        inline void getEnergyPartialGradByCoeffsYaw(Eigen::VectorXd &gdyStem, Eigen::VectorXd &gdyMain, Eigen::VectorXd &gdySub) const{
            // Eigen::VectorXd gdyMS;
            // SteMa.getEnergyPartialGradByCoeffsYaw(gdyMS);
            // gdyStem = gdyMS.head(6 * stemPieces);
            // gdyMain = gdyMS.tail(6 * mainPieces);

            Stem.getEnergyPartialGradByCoeffsYaw(gdyStem);
            Main.getEnergyPartialGradByCoeffsYaw(gdyMain);
            Sub.getEnergyPartialGradByCoeffsYaw(gdySub);
        }

        inline void getEnergyPosPartialGradByTimes(Eigen::VectorXd &gdtStem, Eigen::VectorXd &gdtMain, Eigen::VectorXd &gdtSub) const{
            // Eigen::VectorXd gdtMS;
            // SteMa.getEnergyPosPartialGradByTimes(gdtMS);
            // gdtStem = gdtMS.head(stemPieces);
            // gdtMain = gdtMS.tail(mainPieces);
            Stem.getEnergyPosPartialGradByTimes(gdtStem);
            Main.getEnergyPosPartialGradByTimes(gdtMain);
            Sub.getEnergyPosPartialGradByTimes(gdtSub);
        }

        inline void getEnergyYawPartialGradByTimes(Eigen::VectorXd &gdtStem, Eigen::VectorXd &gdtMain, Eigen::VectorXd &gdtSub) const{
            // Eigen::VectorXd gdtMS;
            // SteMa.getEnergyYawPartialGradByTimes(gdtMS);
            // gdtStem = gdtMS.head(stemPieces);
            // gdtMain = gdtMS.tail(mainPieces);
            Stem.getEnergyYawPartialGradByTimes(gdtStem);
            Main.getEnergyYawPartialGradByTimes(gdtMain);
            Sub.getEnergyYawPartialGradByTimes(gdtSub);
        }

        inline void propogateGrad(
            const Eigen::MatrixX4d &partialGradByCoeffsStem,
            const Eigen::VectorXd &partialGradByTimesStem,
            const Eigen::MatrixX4d &partialGradByCoeffsMain,
            const Eigen::VectorXd &partialGradByTimesMain,
            const Eigen::MatrixX4d &partialGradByCoeffsSub,
            const Eigen::VectorXd &partialGradByTimesSub,
            Eigen::Vector4d &gradByVelStem,
            Eigen::Vector4d &gradByAccStem,
            Eigen::Vector4d &gradByVelMain,
            Eigen::Vector4d &gradByAccMain,
            Eigen::Matrix4Xd &gradByPointsStem,
            Eigen::VectorXd &gradByTimesStem,
            Eigen::Matrix4Xd &gradByPointsMain,
            Eigen::VectorXd &gradByTimesMain,
            Eigen::Matrix4Xd &gradByPointsSub,
            Eigen::VectorXd &gradByTimesSub
        ){
            Eigen::Matrix<double, 3, 4> gradByInitStatesSub, gradByInitStatesMain, gradByTermStatesStem, gradByTermStatesMain;
            Sub.propogateGradInit(partialGradByCoeffsSub, partialGradByTimesSub, gradByPointsSub, gradByTimesSub, gradByInitStatesSub);
            Main.propogateGradInitTerm(partialGradByCoeffsMain, partialGradByTimesMain, gradByPointsMain, gradByTimesMain, gradByInitStatesMain, gradByTermStatesMain);
            Stem.propogateGradTerm(partialGradByCoeffsStem, partialGradByTimesStem, gradByPointsStem, gradByTimesStem, gradByTermStatesStem);
            gradByVelStem = gradByInitStatesSub.row(1).transpose();
            gradByVelStem += gradByTermStatesStem.row(1).transpose();
            gradByVelStem += gradByInitStatesMain.row(1).transpose();
            gradByAccStem = gradByInitStatesSub.row(2).transpose();
            gradByAccStem += gradByTermStatesStem.row(2).transpose();
            gradByAccStem += gradByInitStatesMain.row(2).transpose();
            gradByVelMain  = gradByTermStatesMain.row(1).transpose();
            gradByAccMain  = gradByTermStatesMain.row(2).transpose();
            // Eigen::MatrixX4d partialGradByCoeffsSM((stemPieces + mainPieces)*6, 4);
            // Eigen::VectorXd partialGradByTimesSM(stemPieces + mainPieces);
            // Eigen::Matrix4Xd gradByPointsSM;
            // Eigen::VectorXd gradByTimesSM;
            // partialGradByCoeffsSM.topRows(stemPieces*6) = partialGradByCoeffsStem;
            // partialGradByCoeffsSM.bottomRows(mainPieces*6) = partialGradByCoeffsMain;
            // partialGradByCoeffsSM.block(6 * (stemPieces + 1), 0, 2, 4) += gradByInitStates.block(0, 0, 2, 4);
            // partialGradByCoeffsSM.block(6 * (stemPieces + 1) + 2, 0, 1, 4) += 2*gradByInitStates.block(2, 0, 1, 4);
            // partialGradByTimesSM.head(stemPieces) = partialGradByTimesStem;
            // partialGradByTimesSM.tail(mainPieces) = partialGradByTimesMain;
            // SteMa.propogateGrad(partialGradByCoeffsSM, partialGradByTimesSM, gradByPointsSM, gradByTimesSM);
            // gradByPointsStem = gradByPointsSM.leftCols(stemPieces - 1);
            // gradByPointsMain = gradByPointsSM.rightCols(mainPieces - 1);
        }

        inline void propogateGrad(
            const Eigen::MatrixX4d &partialGradByCoeffsStem,
            const Eigen::VectorXd &partialGradByTimesStem,
            const Eigen::MatrixX4d &partialGradByCoeffsMain,
            const Eigen::VectorXd &partialGradByTimesMain,
            const Eigen::MatrixX4d &partialGradByCoeffsSub,
            const Eigen::VectorXd &partialGradByTimesSub,
            Eigen::Vector4d &gradByVelStem,
            Eigen::Vector4d &gradByAccStem,
            Eigen::Matrix4Xd &gradByPointsStem,
            Eigen::VectorXd &gradByTimesStem,
            Eigen::Matrix4Xd &gradByPointsMain,
            Eigen::VectorXd &gradByTimesMain,
            Eigen::Matrix4Xd &gradByPointsSub,
            Eigen::VectorXd &gradByTimesSub
        ){
            Eigen::Matrix<double, 3, 4> gradByInitStatesSub, gradByInitStatesMain, gradByTermStatesStem, gradByTermStatesMain;
            Sub.propogateGradInit(partialGradByCoeffsSub, partialGradByTimesSub, gradByPointsSub, gradByTimesSub, gradByInitStatesSub);
            Main.propogateGradInit(partialGradByCoeffsMain, partialGradByTimesMain, gradByPointsMain, gradByTimesMain, gradByInitStatesMain);
            Stem.propogateGradTerm(partialGradByCoeffsStem, partialGradByTimesStem, gradByPointsStem, gradByTimesStem, gradByTermStatesStem);
            gradByVelStem = gradByInitStatesSub.row(1).transpose();
            gradByVelStem += gradByTermStatesStem.row(1).transpose();
            gradByVelStem += gradByInitStatesMain.row(1).transpose();
            gradByAccStem = gradByInitStatesSub.row(2).transpose();
            gradByAccStem += gradByTermStatesStem.row(2).transpose();
            gradByAccStem += gradByInitStatesMain.row(2).transpose();

            // Eigen::MatrixX4d partialGradByCoeffsSM((stemPieces + mainPieces)*6, 4);
            // Eigen::VectorXd partialGradByTimesSM(stemPieces + mainPieces);
            // Eigen::Matrix4Xd gradByPointsSM;
            // Eigen::VectorXd gradByTimesSM;
            // partialGradByCoeffsSM.topRows(stemPieces*6) = partialGradByCoeffsStem;
            // partialGradByCoeffsSM.bottomRows(mainPieces*6) = partialGradByCoeffsMain;
            // partialGradByCoeffsSM.block(6 * (stemPieces + 1), 0, 2, 4) += gradByInitStates.block(0, 0, 2, 4);
            // partialGradByCoeffsSM.block(6 * (stemPieces + 1) + 2, 0, 1, 4) += 2*gradByInitStates.block(2, 0, 1, 4);
            // partialGradByTimesSM.head(stemPieces) = partialGradByTimesStem;
            // partialGradByTimesSM.tail(mainPieces) = partialGradByTimesMain;
            // SteMa.propogateGrad(partialGradByCoeffsSM, partialGradByTimesSM, gradByPointsSM, gradByTimesSM);
            // gradByPointsStem = gradByPointsSM.leftCols(stemPieces - 1);
            // gradByPointsMain = gradByPointsSM.rightCols(mainPieces - 1);
        }


    };


}

#endif
