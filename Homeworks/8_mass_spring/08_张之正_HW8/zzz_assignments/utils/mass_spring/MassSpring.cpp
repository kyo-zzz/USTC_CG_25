#include "MassSpring.h"
#include <iostream>

namespace USTC_CG::mass_spring {
MassSpring::MassSpring(const Eigen::MatrixXd& X, const EdgeSet& E)
{
    this->X = this->init_X = X;
    this->vel = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    this->E = E;

    std::cout << "number of edges: " << E.size() << std::endl;
    std::cout << "init mass spring" << std::endl;

    // Compute the rest pose edge length
    for (const auto& e : E) {
        Eigen::Vector3d x0 = X.row(e.first);
        Eigen::Vector3d x1 = X.row(e.second);
        this->E_rest_length.push_back((x0 - x1).norm());
    }

    // Initialize the mask for Dirichlet boundary condition
    dirichlet_bc_mask.resize(X.rows(), false);

    // (HW_TODO) Fix two vertices, feel free to modify this 
    unsigned n_fix = sqrt(X.rows());  // Here we assume the cloth is square
    dirichlet_bc_mask[0] = true;
    dirichlet_bc_mask[n_fix - 1] = true;
}

void MassSpring::step()
{
    Eigen::Vector3d acceleration_ext = gravity + wind_ext_acc;

    unsigned n_vertices = X.rows();

    // The reason to not use 1.0 as mass per vertex: the cloth gets heavier as we increase the resolution
    double mass_per_vertex =
        mass / n_vertices; 

    //----------------------------------------------------
    // (HW Optional) Bonus part: Sphere collision
    Eigen::MatrixXd acceleration_collision =
        getSphereCollisionForce(sphere_center.cast<double>(), sphere_radius);
    //----------------------------------------------------

    if (time_integrator == IMPLICIT_EULER) {
        // Implicit Euler
        TIC(step)

        // (HW TODO) 
        // auto H_elastic = computeHessianSparse(stiffness);  // size = [nx3, nx3]
        auto H = computeHessianSparse(stiffness);
        // compute Y 
        MatrixXd Y = X + h * vel;
        Y.rowwise() += h * h * acceleration_ext.transpose();

        if (enable_sphere_collision) {
            Y += h * h * acceleration_collision;
        }
        // Solve Newton's search direction with linear solver 
        MatrixXd grad_g = mass_per_vertex * (X - Y) / h / h + computeGrad(stiffness);
        VectorXd grad_g_flatten = flatten(grad_g);
        for (int i = 0; i < X.rows(); i++)
        {
            if (dirichlet_bc_mask[i] == 1)
            {
                grad_g_flatten[3 * i + 0] = 0;
                grad_g_flatten[3 * i + 1] = 0;
                grad_g_flatten[3 * i + 2] = 0;
            }
        }        
        // update X and vel 
        SparseLU<SparseMatrix<double>> solver;
        solver.compute(H);
        VectorXd deltaX_flatten = solver.solve(-grad_g_flatten);
        MatrixXd deltaX = unflatten(deltaX_flatten);
        X += deltaX;
        vel = deltaX / h;
        // **Delete the following two lines**
        //vel = 0.03f * Eigen::MatrixXd::Ones(X.rows(), X.cols());
        //X += vel * h;

        TOC(step)
    }
    else if (time_integrator == SEMI_IMPLICIT_EULER) {

        // Semi-implicit Euler
        Eigen::MatrixXd acceleration = -computeGrad(stiffness) / mass_per_vertex;
        acceleration.rowwise() += acceleration_ext.transpose();

        // -----------------------------------------------
        // (HW Optional)
        if (enable_sphere_collision) {
            acceleration += acceleration_collision;
        }
        // -----------------------------------------------

        // (HW TODO): Implement semi-implicit Euler time integration
        vel += h * acceleration;// Update velocity
        for (int i = 0; i < n_vertices; i++) {
            if (dirichlet_bc_mask[i] == true) {
                vel.row(i).setZero();
            }
        }
        X += vel * h; // Update position
        vel *= damping; // Damping
        // Update X and vel 
        
    }
    else {
        std::cerr << "Unknown time integrator!" << std::endl;
        return;
    }
}

// There are different types of mass spring energy:
// For this homework we will adopt Prof. Huamin Wang's energy definition introduced in GAMES103
// course Lecture 2 E = 0.5 * stiffness * sum_{i=1}^{n} (||x_i - x_j|| - l)^2 There exist other
// types of energy definition, e.g., Prof. Minchen Li's energy definition
// https://www.cs.cmu.edu/~15769-f23/lec/3_Mass_Spring_Systems.pdf
double MassSpring::computeEnergy(double stiffness)
{
    double sum = 0.;
    unsigned i = 0;
    for (const auto& e : E) {
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        sum += 0.5 * stiffness * std::pow((diff.norm() - l), 2);
        i++;
    }
    return sum;
}

Eigen::MatrixXd MassSpring::computeGrad(double stiffness)
{
    Eigen::MatrixXd g = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    unsigned i = 0;
    for (const auto& e : E) {
        // --------------------------------------------------
        // (HW TODO): Implement the gradient computation
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        auto grad_norm = stiffness * (diff.norm() - l);
        auto grad_dir = diff / diff.norm();
        g.row(e.first) += grad_norm * grad_dir;
        g.row(e.second) += -grad_norm * grad_dir;
        // --------------------------------------------------
        i++;
    }
    return g;
}

Eigen::SparseMatrix<double> MassSpring::computeHessianSparse(double stiffness)
{
    unsigned n_vertices = X.rows();
    double mass_per_vertex = mass / n_vertices;
    Eigen::SparseMatrix<double> H(3 * n_vertices, 3 * n_vertices);
    std::vector<Eigen::Triplet<double>> tripletList;

    unsigned i = 0;

    for (const auto& e : E) {
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        auto x_n = diff.norm();
        auto x = diff.transpose() * diff;
        auto I = Eigen::MatrixXd::Identity(3, 3);

        Eigen::MatrixXd x_normalized = x / (x_n * x_n);

        Eigen::MatrixXd Hi = stiffness * ((1 - (l / x_n)) * (I - x_normalized) + x_normalized);
        if (l > x_n) 
        {
            Hi = stiffness * x_normalized;
        }

        bool f = 1, s = 1;
        if (dirichlet_bc_mask[e.first] == 1)
            f = 0;
        if (dirichlet_bc_mask[e.second] == 1)
            s = 0;

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                int row_idx1 = 3 * e.first + r;
                int col_idx1 = 3 * e.first + c;
                int row_idx2 = 3 * e.second + r;
                int col_idx2 = 3 * e.second + c;

                tripletList.emplace_back(row_idx1, col_idx1, f * Hi(r, c));
                tripletList.emplace_back(row_idx2, col_idx2, s * Hi(r, c));
                tripletList.emplace_back(row_idx1, col_idx2, -f * Hi(r, c));
                tripletList.emplace_back(row_idx2, col_idx1, -s * Hi(r, c));
            }
        }
        i++;
    }

    for (int k = 0; k < X.rows(); k++)
    {
        if (dirichlet_bc_mask[k] == 0)
        {
            tripletList.emplace_back(3 * k + 0, 3 * k + 0, mass_per_vertex / h / h);
            tripletList.emplace_back(3 * k + 1, 3 * k + 1, mass_per_vertex / h / h);
            tripletList.emplace_back(3 * k + 2, 3 * k + 2, mass_per_vertex / h / h);
        }
        else
        {
            tripletList.emplace_back(3 * k + 0, 3 * k + 0, 1);
            tripletList.emplace_back(3 * k + 1, 3 * k + 1, 1);
            tripletList.emplace_back(3 * k + 2, 3 * k + 2, 1);
        }
    }
    
    H.setFromTriplets(tripletList.begin(), tripletList.end());
    H.makeCompressed();
    return H;
}


bool MassSpring::checkSPD(const Eigen::SparseMatrix<double>& A)
{
    // Eigen::SimplicialLDLT<SparseMatrix_d> ldlt(A);
    // return ldlt.info() == Eigen::Success;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
    auto eigen_values = es.eigenvalues();
    return eigen_values.minCoeff() >= 1e-10;
}

void MassSpring::reset()
{
    std::cout << "reset" << std::endl;
    this->X = this->init_X;
    this->vel.setZero();
}

// ----------------------------------------------------------------------------------
// (HW Optional) Bonus part
Eigen::MatrixXd MassSpring::getSphereCollisionForce(Eigen::Vector3d center, double radius)
{
    Eigen::MatrixXd force = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    for (int i = 0; i < X.rows(); i++) {
       // (HW Optional) Implement penalty-based force here 
    }
    return force;
}
// ----------------------------------------------------------------------------------
 
bool MassSpring::set_dirichlet_bc_mask(const std::vector<bool>& mask)
{
	if (mask.size() == X.rows())
	{
		dirichlet_bc_mask = mask;
		return true;
	}
	else
		return false;
}

bool MassSpring::update_dirichlet_bc_vertices(const MatrixXd &control_vertices)
{
   for (int i = 0; i < dirichlet_bc_control_pair.size(); i++)
   {
       int idx = dirichlet_bc_control_pair[i].first;
	   int control_idx = dirichlet_bc_control_pair[i].second;
	   X.row(idx) = control_vertices.row(control_idx);
   }

   return true; 
}

bool MassSpring::init_dirichlet_bc_vertices_control_pair(const MatrixXd &control_vertices,
    const std::vector<bool>& control_mask)
{
    
	if (control_mask.size() != control_vertices.rows())
			return false; 

   // TODO: optimize this part from O(n) to O(1)
   // First, get selected_control_vertices
   std::vector<VectorXd> selected_control_vertices; 
   std::vector<int> selected_control_idx; 
   for (int i = 0; i < control_mask.size(); i++)
   {
       if (control_mask[i])
       {
			selected_control_vertices.push_back(control_vertices.row(i));
            selected_control_idx.push_back(i);
		}
   }

   // Then update mass spring fixed vertices 
   for (int i = 0; i < dirichlet_bc_mask.size(); i++)
   {
       if (dirichlet_bc_mask[i])
       {
           // O(n^2) nearest point search, can be optimized
           // -----------------------------------------
           int nearest_idx = 0;
           double nearst_dist = 1e6; 
           VectorXd X_i = X.row(i);
           for (int j = 0; j < selected_control_vertices.size(); j++)
           {
               double dist = (X_i - selected_control_vertices[j]).norm();
               if (dist < nearst_dist)
               {
				   nearst_dist = dist;
				   nearest_idx = j;
			   }
           }
           //-----------------------------------------
           
		   X.row(i) = selected_control_vertices[nearest_idx];
           dirichlet_bc_control_pair.push_back(std::make_pair(i, selected_control_idx[nearest_idx]));
	   }
   }

   return true; 
}

}  // namespace USTC_CG::node_mass_spring

