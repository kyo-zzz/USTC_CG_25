#include "wcsph.h"
#include <iostream>
using namespace Eigen;

namespace USTC_CG::sph_fluid {

WCSPH::WCSPH(const MatrixXd& X,const Vector3d& box_min,const Vector3d& box_max)
    : SPHBase(X,box_min,box_max)
{}

void WCSPH::compute_density()
{
    const auto& particles = ps_.particles();
    const size_t num_particles = particles.size();
    const double mass = ps_.mass();
    const double h = ps_.h();
    const double density0 = ps_.density0();

    #pragma omp parallel for
    for(size_t i = 0; i < num_particles; ++i) {
        auto& current_particle = particles[i];
        double density = mass * W_zero(h);

        for(const auto& neighbor : current_particle->neighbors()) {
            const Vector3d x_ij = current_particle->x() - neighbor->x();
            density += mass * W(x_ij,h);
        }

        density = std::max(density,density0);
        current_particle->density() = density;

        const double pressure = stiffness_ *
            (std::pow(density / density0,exponent_) - 1.0);
        current_particle->pressure() = std::max(0.0,pressure);
    }
    // -------------------------------------------------------------
    // (HW TODO) Implement the density computation
    // You can also compute pressure in this function 
    // -------------------------------------------------------------
}

void WCSPH::step()
{
    TIC(step)
        // -------------------------------------------------------------
        // (HW TODO) Follow the instruction in documents and PPT, 
        // implement the pipeline of fluid simulation 
        // -------------------------------------------------------------
        // Search neighbors, compute density, advect, solve pressure acceleration, etc. 

        // Update spatial hash grid
        ps_.assign_particles_to_cells();

    // Find neighboring particles
    ps_.search_neighbors();

    // Calculate particle densities and pressures
    compute_density();

    // Compute external forces (gravity, viscosity, etc.)
    compute_non_pressure_acceleration();
    // Compute internal pressure forces
    compute_pressure_gradient_acceleration();

    // Update particle positions and velocities
    advect();

    TOC(step)
}
}  // namespace USTC_CG::node_sph_fluid
