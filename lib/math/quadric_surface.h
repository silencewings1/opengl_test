#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <vector>

using Vec9 = Eigen::Matrix<double, 9, 1>;

class QuadricSurface
{
public:
    QuadricSurface() = default;
    QuadricSurface(const Vec9& coff)
        : A(coff(0))
        , B(coff(1))
        , C(coff(2))
        , D(coff(3) / 2)
        , E(coff(4) / 2)
        , F(coff(5) / 2)
        , G(coff(6) / 2)
        , H(coff(7) / 2)
        , I(coff(8) / 2)
        , J(-1)
        , valid(true)
    {
        Eigen::Matrix4d DELTA;
        DELTA << A, F, E, G,
            F, B, D, H,
            E, D, C, I,
            G, H, I, J;

        Eigen::Matrix3d delta;
        delta << A, F, E, F, B, D, E, D, C;
        std::cout << "coff: " << coff.transpose() << std::endl;
        std::cout << "DELTA: " << DELTA.determinant() << std::endl;
        std::cout << "delta: " << delta.determinant() << std::endl;
    }

    bool Valid() const { return valid; }

    double Value(const Eigen::Vector3d& point) const
    {
        const auto &x = point.x(), &y = point.y(), &z = point.z();
        return A * x * x + B * y * y + C * z * z +
               2 * (D * y * z + E * x * z + F * x * y) +
               2 * (G * x + H * y + I * z) + J;
    }

    Eigen::Vector3d Gradiant(const Eigen::Vector3d& point) const
    {
        const auto &x = point.x(), &y = point.y(), &z = point.z();

        Eigen::Vector3d grad;
        grad.x() = 2 * (A * x + E * z + F * y + G);
        grad.y() = 2 * (B * y + D * z + F * x + H);
        grad.z() = 2 * (C * z + D * y + E * x + I);

        return grad;
    }

private:
    double A, B, C, D, E, F, G, H, I, J;
    bool valid = false;
};

namespace QuadricSurfaceSolver
{

QuadricSurface Solve(const std::vector<Eigen::Vector3d>& data)
{
    if (data.size() < 9)
    {
        std::cout << "input data size less than 9\n";
        return QuadricSurface();
    }

    Eigen::MatrixXd Co = Eigen::MatrixXd::Zero(data.size(), 9);
    for (size_t i = 0; i < data.size(); ++i)
    {
        const auto &x = data[i].x(), &y = data[i].y(), &z = data[i].z();
        Vec9 row_data;
        row_data << x * x, y * y, z * z, y * z, x * z, x * y, x, y, z;
        Co.row(i) = row_data;
    }

    Eigen::Matrix<double, 9, 9> ATA = Co.transpose() * Co;
    Vec9 ATb = Co.transpose() * Eigen::VectorXd::Ones(data.size());

    Vec9 X = ATA.ldlt().solve(ATb);

    return QuadricSurface(X);
}

} // namespace QuadricSurfaceSolver